#include "telemetry_bus.hpp"
#include "esp_log.h"
#include <cassert>

static const char* TAG = "telemetry";

namespace drone {

TelemetryBus& TelemetryBus::instance() {
    static TelemetryBus bus;
    return bus;
}

TelemetryBus::TelemetryBus() {
    mtx_ = xSemaphoreCreateRecursiveMutex();
    assert(mtx_ != nullptr);
}

TelemetryBus::~TelemetryBus() {
    if (mtx_) vSemaphoreDelete(mtx_);
}

bool TelemetryBus::lock() const {
    if (xSemaphoreTakeRecursive(mtx_, kTimeout) == pdTRUE) return true;
    // On Core 1: log at LOGW, continue with stale data rather than blocking.
    ESP_LOGW(TAG, "mutex timeout — stale data");
    return false;
}

void TelemetryBus::unlock() const {
    xSemaphoreGiveRecursive(mtx_);
}

TelemetryBus::Snapshot TelemetryBus::get() const {
    Snapshot s{};
    if (lock()) {
        s = data_;
        unlock();
    }
    return s;
}

void TelemetryBus::update_attitude(float roll, float pitch, float yaw) {
    if (!lock()) return;
    data_.roll  = roll;
    data_.pitch = pitch;
    data_.yaw   = yaw;
    unlock();
}

void TelemetryBus::update_motors(const uint16_t (&duty)[4]) {
    if (!lock()) return;
    for (int i = 0; i < 4; ++i) data_.throttle[i] = duty[i];
    unlock();
}

void TelemetryBus::update_vbat(float v) {
    if (!lock()) return;
    data_.vbat_v = v;
    unlock();
}

void TelemetryBus::update_gps(float lat, float lon, bool fix) {
    if (!lock()) return;
    data_.gps_lat = lat;
    data_.gps_lon = lon;
    data_.gps_fix = fix;
    unlock();
}

void TelemetryBus::update_loop_us(uint32_t us) {
    if (!lock()) return;
    data_.loop_us = us;
    unlock();
}

void TelemetryBus::set_flight_mode(uint8_t mode) {
    if (!lock()) return;
    data_.flight_mode = mode;
    unlock();
}

}  // namespace drone
