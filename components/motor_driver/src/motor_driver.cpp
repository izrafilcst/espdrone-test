#include "motor_driver.hpp"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <cassert>
#include <cstring>

static const char* TAG = "motor";

// GPIO order matches CLAUDE.md §3.3 motor table.
static constexpr int kGpioMap[drone::MotorDriver::kCount] = {
    drone::pin::MOTOR_M1_FRONT_LEFT,   // 38 — CCW
    drone::pin::MOTOR_M2_FRONT_RIGHT,  // 39 — CW
    drone::pin::MOTOR_M3_REAR_RIGHT,   // 40 — CCW
    drone::pin::MOTOR_M4_REAR_LEFT,    // 41 — CW
};

namespace drone {

MotorDriver& MotorDriver::instance() {
    static MotorDriver drv;
    return drv;
}

esp_err_t MotorDriver::init() {
    if (initialized_) return ESP_OK;

    cmd_queue_ = xQueueCreate(4, sizeof(uint16_t) * kCount);
    assert(cmd_queue_ != nullptr);

    esp_err_t ret = init_mcpwm();
    if (ret == ESP_OK) {
        initialized_ = true;
        ESP_LOGI(TAG, "init OK — %d motors @ %lu Hz %lu-bit",
                 kCount, config::MOTOR_PWM_HZ, config::MOTOR_PWM_BITS);
    }
    return ret;
}

esp_err_t MotorDriver::init_mcpwm() {
    // Timer: Group 0, period = MOTOR_DUTY_MAX+1 ticks → ~19.5 kHz @ 160 MHz.
    // Using 160 MHz / (DUTY_MAX+1) ≈ 19531 Hz (< 20 kHz, above audible threshold).
    mcpwm_timer_config_t tcfg{};
    tcfg.group_id      = 0;
    tcfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tcfg.resolution_hz = 160'000'000;
    tcfg.period_ticks  = config::MOTOR_DUTY_MAX + 1;
    tcfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&tcfg, &timer_), TAG, "new_timer");

    // Two operators: oper[0] drives motors 0&1, oper[1] drives motors 2&3.
    for (int op = 0; op < 2; ++op) {
        mcpwm_operator_config_t ocfg{};
        ocfg.group_id = 0;
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&ocfg, &opers_[op]), TAG, "new_oper");
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(opers_[op], timer_), TAG, "connect");
    }

    for (int i = 0; i < kCount; ++i) {
        mcpwm_comparator_config_t ccfg{};
        ccfg.flags.update_cmp_on_tez = true;
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(opers_[i / 2], &ccfg, &cmps_[i]), TAG, "new_cmp");
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(cmps_[i], 0), TAG, "cmp_zero");

        mcpwm_generator_config_t gcfg{};
        gcfg.gen_gpio_num = kGpioMap[i];
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(opers_[i / 2], &gcfg, &gens_[i]), TAG, "new_gen");

        // Active-high PWM: HIGH at timer-zero, LOW at comparator match.
        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_timer_event(
                gens_[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
            TAG, "gen_high");
        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                gens_[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP, cmps_[i], MCPWM_GEN_ACTION_LOW)),
            TAG, "gen_low");
    }

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timer_), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP), TAG, "timer_start");
    return ESP_OK;
}

void MotorDriver::arm() {
    arm_start_ms_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
    armed_        = true;
    ESP_LOGI(TAG, "ARMED — soft-start %lu ms", config::SOFT_START_MS);
}

void MotorDriver::disarm() {
    armed_ = false;
    uint16_t zero[kCount]{};
    apply_duty(zero);
    std::fill(current_, current_ + kCount, uint16_t{0});
    std::fill(target_,  target_  + kCount, uint16_t{0});
    ESP_LOGI(TAG, "DISARMED");
}

void MotorDriver::set_target(const uint16_t (&duty)[kCount]) {
    // Drop-oldest if queue full — set_target must never block the PID loop.
    if (xQueueSend(cmd_queue_, duty, 0) != pdTRUE) {
        uint16_t discard[kCount];
        xQueueReceive(cmd_queue_, discard, 0);
        xQueueSend(cmd_queue_, duty, 0);
    }
}

uint16_t MotorDriver::slew(uint16_t current, uint16_t target) {
    const int32_t delta   = static_cast<int32_t>(target) - static_cast<int32_t>(current);
    const int32_t max_d   = static_cast<int32_t>(config::SLEW_PER_MS_MAX);
    const int32_t clamped = std::clamp(delta, -max_d, max_d);
    return static_cast<uint16_t>(static_cast<int32_t>(current) + clamped);
}

void MotorDriver::apply_duty(const uint16_t (&duty)[kCount]) {
    for (int i = 0; i < kCount; ++i) {
        mcpwm_comparator_set_compare_value(cmps_[i], duty[i]);
    }
}

void MotorDriver::start() {
    xTaskCreatePinnedToCore(task_thunk, "motor_drv", 4096, this,
                            21, &handle_, /*core=*/1);
}

void MotorDriver::task_thunk(void* arg) {
    static_cast<MotorDriver*>(arg)->run();
}

void MotorDriver::run() {
    esp_task_wdt_add(nullptr);
    TickType_t last = xTaskGetTickCount();

    while (true) {
        esp_task_wdt_reset();

        // Drain command queue (take latest available target).
        uint16_t incoming[kCount];
        while (xQueueReceive(cmd_queue_, incoming, 0) == pdTRUE) {
            for (int i = 0; i < kCount; ++i) {
                target_[i] = std::min(incoming[i],
                    static_cast<uint16_t>(config::MOTOR_DUTY_MAX));
            }
        }

        if (armed_) {
            const uint32_t now_ms   = xTaskGetTickCount() * portTICK_PERIOD_MS;
            const uint32_t elapsed  = now_ms - arm_start_ms_;

            uint16_t next[kCount];
            for (int i = 0; i < kCount; ++i) {
                // Soft-start: scale target linearly over SOFT_START_MS (§7.2).
                uint16_t ramp = target_[i];
                if (elapsed < config::SOFT_START_MS) {
                    ramp = static_cast<uint16_t>(
                        (static_cast<uint64_t>(target_[i]) * elapsed) / config::SOFT_START_MS);
                }
                next[i] = slew(current_[i], ramp);
            }

            apply_duty(next);
            for (int i = 0; i < kCount; ++i) current_[i] = next[i];
        }

        xTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}

}  // namespace drone
