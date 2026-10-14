#pragma once

#include <cstdint>

namespace drone::config {

constexpr const char* FIRMWARE_VERSION = "0.2.0-dev";
constexpr const char* TARGET_BOARD     = "esp32-s3-wroom-1 N16R8";

// ----- Loop de voo (CLAUDE.md §4.2) -----
constexpr uint32_t PID_LOOP_HZ        = 1000;
constexpr uint32_t PID_PERIOD_MS      = 1;
constexpr uint32_t STATE_MACHINE_MS   = 10;

// ----- Motores (CLAUDE.md §10) -----
constexpr uint32_t MOTOR_PWM_HZ       = 20000;
constexpr uint32_t MOTOR_PWM_BITS     = 13;
constexpr uint32_t MOTOR_DUTY_MAX     = (1u << MOTOR_PWM_BITS) - 1;

// ----- Soft-start / slew-rate (CLAUDE.md §7.2) -----
constexpr uint32_t SOFT_START_MS      = 800;
constexpr uint32_t SLEW_PER_MS_MAX    = 50;

// ----- Failsafe (CLAUDE.md §1) -----
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;

// ----- Bateria 1S Li-ion (CLAUDE.md §6.1) -----
constexpr float    VBAT_NOMINAL_V     = 4.0f;
constexpr float    VBAT_MIN_V         = 3.3f;
constexpr float    VBAT_MAX_V         = 4.2f;
constexpr float    SAG_COMP_CEILING   = 1.35f;

// ----- Watchdog -----
constexpr uint32_t TWDT_TIMEOUT_S     = 5;
constexpr uint32_t IWDT_TIMEOUT_MS    = 300;

// ----- Telemetria (CLAUDE.md §5.1) -----
constexpr uint32_t TELEMETRY_TX_MS    = 100;
constexpr uint32_t MUTEX_TIMEOUT_MS   = 1;

// ----- Camera / streaming -----
constexpr uint32_t CAMERA_FRAME_MS    = 33;   // ~30 fps
constexpr uint16_t HTTP_STREAM_PORT   = 80;

}
