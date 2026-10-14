#include "imu_mpu6050.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cassert>
#include <cstring>

static const char* TAG = "imu";

// MPU6050 register map (subset used here).
namespace Reg {
    static constexpr uint8_t CONFIG       = 0x1A;
    static constexpr uint8_t GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t INT_ENABLE   = 0x38;
    static constexpr uint8_t INT_STATUS   = 0x3A;
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;  // 14 bytes: accel(6) + temp(2) + gyro(6)
    static constexpr uint8_t USER_CTRL    = 0x6A;
    static constexpr uint8_t PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t WHO_AM_I     = 0x75;
}

namespace drone {

ImuMpu6050& ImuMpu6050::instance() {
    static ImuMpu6050 imu;
    return imu;
}

esp_err_t ImuMpu6050::init(TaskHandle_t notify_target) {
    if (initialized_) return ESP_OK;
    pid_loop_ = notify_target;

    // I2C1 master bus on GPIO 47(SDA)/48(SCL) @ 400 kHz.
    i2c_master_bus_config_t bus_cfg{};
    bus_cfg.i2c_port    = I2C_NUM_1;
    bus_cfg.sda_io_num  = static_cast<gpio_num_t>(pin::IMU_SDA);
    bus_cfg.scl_io_num  = static_cast<gpio_num_t>(pin::IMU_SCL);
    bus_cfg.clk_source  = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = false;  // external 4.7k fitted (§3.3)
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus_), TAG, "bus");

    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = kAddr;
    dev_cfg.scl_speed_hz    = 400'000;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_, &dev_cfg, &dev_), TAG, "dev");

    // Verify WHO_AM_I.
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(read_regs(Reg::WHO_AM_I, &who, 1), TAG, "who_am_i");
    if (who != kAddr) {
        ESP_LOGE(TAG, "unexpected WHO_AM_I: 0x%02X (expected 0x68)", who);
        return ESP_ERR_NOT_FOUND;
    }

    // Wake from sleep, select PLL with X-gyro as clock source (recommended).
    ESP_RETURN_ON_ERROR(write_reg(Reg::PWR_MGMT_1, 0x01), TAG, "pwr");

    // DLPF = 41 Hz BW — rejects motor EMI, phase lag ~4.9 ms (§6.2).
    ESP_RETURN_ON_ERROR(configure_dlpf(), TAG, "dlpf");

    // Full-scale: accel ±2g (0x00), gyro ±250°/s (0x00) — defaults.
    ESP_RETURN_ON_ERROR(write_reg(Reg::GYRO_CONFIG,  0x00), TAG, "gyro_cfg");
    ESP_RETURN_ON_ERROR(write_reg(Reg::ACCEL_CONFIG, 0x00), TAG, "accel_cfg");

    // Enable Data Ready interrupt on INT pin.
    ESP_RETURN_ON_ERROR(write_reg(Reg::INT_ENABLE, 0x01), TAG, "int_en");

    // Configure INT GPIO: rising edge, pull-down.
    gpio_config_t io_cfg{};
    io_cfg.pin_bit_mask = 1ULL << pin::IMU_INT;
    io_cfg.mode         = GPIO_MODE_INPUT;
    io_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_cfg.intr_type    = GPIO_INTR_POSEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "gpio_cfg");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "isr_svc");
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(static_cast<gpio_num_t>(pin::IMU_INT), isr_handler, this),
        TAG, "isr_add");

    initialized_ = true;
    ESP_LOGI(TAG, "MPU6050 OK — WHO_AM_I=0x%02X, DLPF=41Hz", who);
    return ESP_OK;
}

esp_err_t ImuMpu6050::configure_dlpf() {
    // CONFIG register bits[2:0] = DLPF_CFG.
    // 0x03 → accel 44 Hz BW / gyro 41 Hz BW (datasheet Table 10).
    return write_reg(Reg::CONFIG, 0x03);
}

esp_err_t ImuMpu6050::calibrate(uint16_t samples) {
    ESP_LOGI(TAG, "calibrating — %u samples, keep drone flat and still", samples);

    int32_t ax_sum{}, ay_sum{}, az_sum{};
    int32_t gx_sum{}, gy_sum{}, gz_sum{};

    for (uint16_t i = 0; i < samples; ++i) {
        uint8_t buf[14];
        if (read_regs(Reg::ACCEL_XOUT_H, buf, 14) != ESP_OK) continue;

        ax_sum += static_cast<int16_t>((buf[0] << 8) | buf[1]);
        ay_sum += static_cast<int16_t>((buf[2] << 8) | buf[3]);
        az_sum += static_cast<int16_t>((buf[4] << 8) | buf[5]);
        gx_sum += static_cast<int16_t>((buf[8]  << 8) | buf[9]);
        gy_sum += static_cast<int16_t>((buf[10] << 8) | buf[11]);
        gz_sum += static_cast<int16_t>((buf[12] << 8) | buf[13]);

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    offsets_.ax = static_cast<int16_t>(ax_sum / samples);
    offsets_.ay = static_cast<int16_t>(ay_sum / samples);
    offsets_.az = static_cast<int16_t>(az_sum / samples - (1.0f / kAccelScale));  // subtract 1g
    offsets_.gx = static_cast<int16_t>(gx_sum / samples);
    offsets_.gy = static_cast<int16_t>(gy_sum / samples);
    offsets_.gz = static_cast<int16_t>(gz_sum / samples);

    ESP_LOGI(TAG, "offsets: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d",
             offsets_.ax, offsets_.ay, offsets_.az,
             offsets_.gx, offsets_.gy, offsets_.gz);
    return ESP_OK;
}

ImuData ImuMpu6050::read() {
    uint8_t buf[14];
    if (read_regs(Reg::ACCEL_XOUT_H, buf, 14) != ESP_OK) return latest_;

    const int16_t ax = static_cast<int16_t>((buf[0]  << 8) | buf[1])  - offsets_.ax;
    const int16_t ay = static_cast<int16_t>((buf[2]  << 8) | buf[3])  - offsets_.ay;
    const int16_t az = static_cast<int16_t>((buf[4]  << 8) | buf[5])  - offsets_.az;
    const int16_t tmp = static_cast<int16_t>((buf[6]  << 8) | buf[7]);
    const int16_t gx = static_cast<int16_t>((buf[8]  << 8) | buf[9])  - offsets_.gx;
    const int16_t gy = static_cast<int16_t>((buf[10] << 8) | buf[11]) - offsets_.gy;
    const int16_t gz = static_cast<int16_t>((buf[12] << 8) | buf[13]) - offsets_.gz;

    latest_.accel_x     = ax  * kAccelScale;
    latest_.accel_y     = ay  * kAccelScale;
    latest_.accel_z     = az  * kAccelScale;
    latest_.gyro_x      = gx  * kGyroScale;
    latest_.gyro_y      = gy  * kGyroScale;
    latest_.gyro_z      = gz  * kGyroScale;
    latest_.temp_c      = static_cast<float>(tmp) / 340.0f + 36.53f;
    latest_.timestamp_us = static_cast<uint32_t>(esp_timer_get_time());

    return latest_;
}

void ImuMpu6050::start(TaskHandle_t pid_loop_handle) {
    pid_loop_ = pid_loop_handle;
    xTaskCreatePinnedToCore(task_thunk, "imu_read", 6144, this,
                            23, &sensor_task_, /*core=*/1);
}

void ImuMpu6050::task_thunk(void* arg) {
    static_cast<ImuMpu6050*>(arg)->run();
}

void ImuMpu6050::run() {
    esp_task_wdt_add(nullptr);

    while (true) {
        esp_task_wdt_reset();
        // Block until ISR fires (Data Ready @ 1 kHz default ODR).
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));

        const ImuData d = read();

        // Notify PID loop that fresh data is available.
        if (pid_loop_) xTaskNotifyGive(pid_loop_);

        (void)d;  // data picked up via read() from PID task context
    }
}

// ISR: minimal — only signals the sensor task, no I2C in interrupt context.
void IRAM_ATTR ImuMpu6050::isr_handler(void* arg) {
    auto* self = static_cast<ImuMpu6050*>(arg);
    BaseType_t woken = pdFALSE;
    if (self->sensor_task_) {
        vTaskNotifyGiveFromISR(self->sensor_task_, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

esp_err_t ImuMpu6050::write_reg(uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev_, buf, 2, 10);
}

esp_err_t ImuMpu6050::read_regs(uint8_t reg, uint8_t* out, size_t len) {
    return i2c_master_transmit_receive(dev_, &reg, 1, out, len, 10);
}

}  // namespace drone
