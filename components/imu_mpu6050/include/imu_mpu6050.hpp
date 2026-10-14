#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "pinmap.hpp"

namespace drone {

struct ImuData {
    float    accel_x, accel_y, accel_z;  // m/s²
    float    gyro_x,  gyro_y,  gyro_z;   // rad/s
    float    temp_c;
    uint32_t timestamp_us;                // esp_timer_get_time()
};

// MPU6050 driver on I2C1 @ 400 kHz, INT on GPIO2 (CLAUDE.md §3.3).
// ISR triggers TaskNotifyGiveFromISR to task_sensor_read (Core 1, prio 23).
// DLPF set to 41 Hz to reject EMI without introducing excessive phase lag (§6.2).
class ImuMpu6050 {
public:
    static ImuMpu6050& instance();

    // Call once from main. notify_target = handle of task_sensor_read.
    esp_err_t init(TaskHandle_t notify_target = nullptr);

    // Offline calibration: average SAMPLES readings with motors off.
    // Stores offsets in DRAM — survives task restarts, not reboots.
    esp_err_t calibrate(uint16_t samples = 2000);

    // Read latest sample. Call from task_sensor_read after TaskNotify.
    ImuData read();

    // Spawn task_sensor_read on Core 1 prio 23.
    void start(TaskHandle_t pid_loop_handle);

private:
    ImuMpu6050() = default;
    ImuMpu6050(const ImuMpu6050&) = delete;
    ImuMpu6050& operator=(const ImuMpu6050&) = delete;

    esp_err_t write_reg(uint8_t reg, uint8_t val);
    esp_err_t read_regs(uint8_t reg, uint8_t* buf, size_t len);
    esp_err_t configure_dlpf();  // DLPF_CFG = 0x03 → 41 Hz BW

    static void IRAM_ATTR isr_handler(void* arg);
    static void task_thunk(void* arg);
    void run();

    struct Offsets {
        int16_t ax{}, ay{}, az{};
        int16_t gx{}, gy{}, gz{};
    };

    // Full-scale ±2g / ±250°/s (MPU6050 power-on defaults).
    static constexpr float kAccelScale = 9.81f / 16384.0f;
    static constexpr float kGyroScale  = (1.0f / 131.0f) * (3.14159265f / 180.0f);
    static constexpr uint8_t kAddr     = 0x68;  // AD0 = GND

    i2c_master_bus_handle_t  bus_{nullptr};
    i2c_master_dev_handle_t  dev_{nullptr};

    Offsets      offsets_{};
    ImuData      latest_{};
    bool         initialized_{false};
    TaskHandle_t sensor_task_{nullptr};
    TaskHandle_t pid_loop_{nullptr};
};

}  // namespace drone
