#pragma once
//
// pinmap.hpp — UNICA fonte de pinagem do projeto.
// Espelha a tabela rigida em CLAUDE.md §3.3.
//
// REGRA INVIOLAVEL: GPIO 26..37 sao reservados pelo modulo N16R8
//   - 26..32: Flash interno QIO
//   - 33..37: PSRAM Octal (8 linhas de dados)
// Qualquer escrita nesses GPIOs causa corrupcao de PSRAM e crash imediato.
// O linter tools/lint_pinmap.py rejeita PRs que reintroduzam esses pinos.
//

#include <cstdint>

namespace drone::pin {

using gpio_t = int;

// ===== Motores — MCPWM Group 0 (PWM 20 kHz, 13-bit) =====
constexpr gpio_t MOTOR_M1_FRONT_LEFT  = 38;  // MCPWM0_OPR0_A — CCW
constexpr gpio_t MOTOR_M2_FRONT_RIGHT = 39;  // MCPWM0_OPR0_B — CW
constexpr gpio_t MOTOR_M3_REAR_RIGHT  = 40;  // MCPWM0_OPR1_A — CCW
constexpr gpio_t MOTOR_M4_REAR_LEFT   = 41;  // MCPWM0_OPR1_B — CW

// ===== Camera OV2640 — DVP paralelo 8-bit =====
constexpr gpio_t CAM_XCLK   = 15;  // LEDC ch0 @ 20 MHz
constexpr gpio_t CAM_SIOD   = 4;   // SCCB SDA (I2C0)
constexpr gpio_t CAM_SIOC   = 5;   // SCCB SCL (I2C0)
constexpr gpio_t CAM_VSYNC  = 6;
constexpr gpio_t CAM_HREF   = 7;
constexpr gpio_t CAM_PCLK   = 13;
constexpr gpio_t CAM_Y2_D0  = 11;
constexpr gpio_t CAM_Y3_D1  = 9;
constexpr gpio_t CAM_Y4_D2  = 8;
constexpr gpio_t CAM_Y5_D3  = 10;
constexpr gpio_t CAM_Y6_D4  = 12;
constexpr gpio_t CAM_Y7_D5  = 18;
constexpr gpio_t CAM_Y8_D6  = 17;
constexpr gpio_t CAM_Y9_D7  = 16;
constexpr gpio_t CAM_PWDN   = -1;  // NC — reset por software
constexpr gpio_t CAM_RESET  = -1;  // NC

// ===== IMU MPU6050 — I2C1 @ 400 kHz =====
constexpr gpio_t IMU_SDA    = 47;  // pull-up 4.7k p/ 3.3V + RC 100Ω/100pF (§6.2)
constexpr gpio_t IMU_SCL    = 48;
constexpr gpio_t IMU_INT    = 2;   // data-ready @ 1 kHz, edge rising

// ===== GPS NEO-6M — UART2 @ 9600 (default) =====
constexpr gpio_t GPS_TX     = 42;  // ESP -> GPS (config UBX opcional)
constexpr gpio_t GPS_RX     = 14;  // GPS -> ESP (NMEA RMC/GGA)

// ===== Energia =====
constexpr gpio_t VBAT_SENSE = 1;   // ADC1_CH0 — divisor 100k:33k

// ===== Indicador de status =====
constexpr gpio_t LED_STATUS = 21;  // RMT WS2812 on-board

// ===== Reservado (futuro) =====
constexpr gpio_t AUX_GPIO   = 3;   // estavel apos boot (strap JTAG-source)

// --------------------------------------------------------------------------
// Asserções de compile-time: nenhum pino acima cai no range proibido 26..37.
// Se um GPIO for alterado para um valor proibido, o build quebra aqui.
// --------------------------------------------------------------------------
namespace detail {
constexpr bool is_safe(gpio_t p) {
    return p < 0 || p < 26 || p > 37;
}
}  // namespace detail

static_assert(detail::is_safe(MOTOR_M1_FRONT_LEFT),  "GPIO conflita com Flash/PSRAM");
static_assert(detail::is_safe(MOTOR_M2_FRONT_RIGHT), "GPIO conflita com Flash/PSRAM");
static_assert(detail::is_safe(MOTOR_M3_REAR_RIGHT),  "GPIO conflita com Flash/PSRAM");
static_assert(detail::is_safe(MOTOR_M4_REAR_LEFT),   "GPIO conflita com Flash/PSRAM");
static_assert(detail::is_safe(CAM_XCLK)  && detail::is_safe(CAM_SIOD) && detail::is_safe(CAM_SIOC),
              "Camera control conflita com Flash/PSRAM");
static_assert(detail::is_safe(CAM_VSYNC) && detail::is_safe(CAM_HREF) && detail::is_safe(CAM_PCLK),
              "Camera sync conflita com Flash/PSRAM");
static_assert(detail::is_safe(CAM_Y2_D0) && detail::is_safe(CAM_Y3_D1) &&
              detail::is_safe(CAM_Y4_D2) && detail::is_safe(CAM_Y5_D3) &&
              detail::is_safe(CAM_Y6_D4) && detail::is_safe(CAM_Y7_D5) &&
              detail::is_safe(CAM_Y8_D6) && detail::is_safe(CAM_Y9_D7),
              "Camera data conflita com Flash/PSRAM");
static_assert(detail::is_safe(IMU_SDA) && detail::is_safe(IMU_SCL) && detail::is_safe(IMU_INT),
              "IMU conflita com Flash/PSRAM");
static_assert(detail::is_safe(GPS_TX) && detail::is_safe(GPS_RX),
              "GPS conflita com Flash/PSRAM");
static_assert(detail::is_safe(VBAT_SENSE), "VBAT_SENSE conflita com Flash/PSRAM");
static_assert(detail::is_safe(LED_STATUS), "LED_STATUS conflita com Flash/PSRAM");
static_assert(detail::is_safe(AUX_GPIO),   "AUX conflita com Flash/PSRAM");

}  // namespace drone::pin
