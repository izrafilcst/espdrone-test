#pragma once

#include <cstdint>
#include <algorithm>
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "app_config.hpp"
#include "pinmap.hpp"

namespace drone {

// MCPWM 4-channel motor driver with soft-start and slew-rate limiting.
// Must run on Core 1 (prio 21). Owned by task_motor_drive (CLAUDE.md §4.2).
class MotorDriver {
public:
    static constexpr uint8_t kCount = 4;

    static MotorDriver& instance();

    // Call once from main before spawning tasks.
    esp_err_t init();

    // Spawn task_motor_drive on Core 1 prio 21.
    void start();

    // Thread-safe. Called from task_pid_loop (Core 1).
    // Drops oldest command if queue is full — never blocks.
    void set_target(const uint16_t (&duty)[kCount]);

    // Arm: enable PWM output with soft-start ramp (SOFT_START_MS).
    void arm();

    // Disarm: immediate cut to zero duty.
    void disarm();

    bool is_armed() const { return armed_; }

private:
    MotorDriver() = default;
    MotorDriver(const MotorDriver&) = delete;
    MotorDriver& operator=(const MotorDriver&) = delete;

    esp_err_t init_mcpwm();
    void apply_duty(const uint16_t (&duty)[kCount]);

    // Slew-rate limit: max SLEW_PER_MS_MAX units per 1 ms tick.
    static uint16_t slew(uint16_t current, uint16_t target);

    static void task_thunk(void* arg);
    void run();

    // MCPWM handles — allocated in init_mcpwm(), never freed at runtime.
    mcpwm_timer_handle_t timer_{nullptr};
    mcpwm_oper_handle_t  opers_[2]{};
    mcpwm_cmpr_handle_t  cmps_[kCount]{};
    mcpwm_gen_handle_t   gens_[kCount]{};

    uint16_t     target_[kCount]{};
    uint16_t     current_[kCount]{};
    bool         armed_{false};
    uint32_t     arm_start_ms_{0};
    bool         initialized_{false};
    TaskHandle_t handle_{nullptr};
    QueueHandle_t cmd_queue_{nullptr};  // depth 4, drop-oldest on overflow
};

}  // namespace drone
