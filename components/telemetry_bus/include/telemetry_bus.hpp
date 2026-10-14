#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "app_config.hpp"

namespace drone {

// Thread-safe singleton shared state bus (CLAUDE.md §5.1).
// Acquisition order when nesting: mtx_telemetry → mtx_motor → mtx_gps.
// Mutex timeout = MUTEX_TIMEOUT_MS (1 ms); on failure returns stale data.
class TelemetryBus {
public:
    struct Snapshot {
        float    roll, pitch, yaw;   // degrees
        float    altitude_m;
        float    vbat_v;
        float    gps_lat, gps_lon;
        uint16_t throttle[4];        // duty 0..8191 (13-bit, CLAUDE.md §10)
        uint32_t loop_us;            // last PID cycle duration
        uint8_t  flight_mode;
        bool     gps_fix;
    };

    static TelemetryBus& instance();

    Snapshot get() const;

    void update_attitude(float roll, float pitch, float yaw);
    void update_motors(const uint16_t (&duty)[4]);
    void update_vbat(float v);
    void update_gps(float lat, float lon, bool fix);
    void update_loop_us(uint32_t us);
    void set_flight_mode(uint8_t mode);

private:
    TelemetryBus();
    ~TelemetryBus();
    TelemetryBus(const TelemetryBus&) = delete;
    TelemetryBus& operator=(const TelemetryBus&) = delete;

    static constexpr TickType_t kTimeout =
        pdMS_TO_TICKS(drone::config::MUTEX_TIMEOUT_MS);

    bool lock() const;
    void unlock() const;

    mutable SemaphoreHandle_t mtx_{nullptr};
    Snapshot data_{};
};

}  // namespace drone
