// =============================================================================
// diag_i2c_scanner.cpp — Firmware standalone Fase 0.1
//
// Objetivo: provar bus I2C1 e detectar MPU6050 (0x68 ou 0x69 com AD0=VCC).
// Critterio de PASS: WHO_AM_I (reg 0x75) == 0x68 estavel por 30 s.
//
// Pinagem espelha CLAUDE.md §3.3 (IMU_SDA=47, IMU_SCL=48, I2C_NUM_1).
// "Codigo todo em main.cpp para facilitar bring-up" — sem componentes.
//
// Driver: ESP-IDF 5.2+ i2c_master (i2c_master_probe + i2c_master_transmit_receive).
// =============================================================================

#include <cinttypes>
#include <cstdint>
#include <cstdio>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "i2c_scan";

// ---- Bus (espelha pinmap.hpp / CLAUDE.md §3.3) ----
constexpr i2c_port_t I2C_PORT     = I2C_NUM_1;
constexpr gpio_num_t SDA_PIN      = static_cast<gpio_num_t>(47);
constexpr gpio_num_t SCL_PIN      = static_cast<gpio_num_t>(48);
constexpr uint32_t   I2C_FREQ_HZ  = 400000;

// ---- Probe / ciclo ----
constexpr int     PROBE_TIMEOUT_MS = 100;
constexpr int     CYCLE_PERIOD_MS  = 2000;
constexpr uint8_t SCAN_LO          = 0x03;
constexpr uint8_t SCAN_HI          = 0x77;

// ---- MPU6050 ----
constexpr uint8_t MPU_ADDR_PRIMARY = 0x68;  // AD0 = GND
constexpr uint8_t MPU_ADDR_ALT_AD0 = 0x69;  // AD0 = VCC
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU_WHO_AM_I_OK  = 0x68;

i2c_master_bus_handle_t g_bus = nullptr;

// -------------------------------------------------------------------------
esp_err_t bus_init()
{
    i2c_master_bus_config_t cfg{};
    cfg.i2c_port                     = I2C_PORT;
    cfg.sda_io_num                   = SDA_PIN;
    cfg.scl_io_num                   = SCL_PIN;
    cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt            = 7;
    cfg.intr_priority                = 0;
    cfg.trans_queue_depth            = 0;  // sync-only
    cfg.flags.enable_internal_pullup = true;  // EXTERNO 4k7 ainda obrigatorio (§6.2)

    return i2c_new_master_bus(&cfg, &g_bus);
}

// -------------------------------------------------------------------------
esp_err_t read_register(uint8_t addr, uint8_t reg, uint8_t& out)
{
    i2c_device_config_t dcfg{};
    dcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dcfg.device_address  = addr;
    dcfg.scl_speed_hz    = I2C_FREQ_HZ;

    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t err = i2c_master_bus_add_device(g_bus, &dcfg, &dev);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_transmit_receive(dev, &reg, 1, &out, 1, PROBE_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev);
    return err;
}

// -------------------------------------------------------------------------
void print_detect_table(const bool found[128])
{
    // Estilo i2cdetect: header + 8 linhas de 16 colunas (0x00..0x7F).
    std::printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (uint8_t row = 0; row < 8; ++row) {
        std::printf("%02x: ", static_cast<unsigned>(row * 16u));
        for (uint8_t col = 0; col < 16; ++col) {
            const uint8_t addr = static_cast<uint8_t>(row * 16u + col);
            if (addr < SCAN_LO || addr > SCAN_HI) {
                std::printf("   ");
            } else if (found[addr]) {
                std::printf("%02x ", addr);
            } else {
                std::printf("-- ");
            }
        }
        std::printf("\n");
    }
}

// -------------------------------------------------------------------------
void log_troubleshooting()
{
    ESP_LOGE(TAG, "==== FAIL: nenhum dispositivo I2C respondeu ====");
    ESP_LOGE(TAG, "Checklist de bring-up (CLAUDE.md §6.2):");
    ESP_LOGE(TAG, "  [1] Pull-ups 4.7k em SDA(GPIO%d) e SCL(GPIO%d) -> 3V3 (CRITICO)",
             SDA_PIN, SCL_PIN);
    ESP_LOGE(TAG, "  [2] VCC do MPU6050 em 3.3V (NUNCA 5V no S3)");
    ESP_LOGE(TAG, "  [3] GND comum entre MCU e sensor");
    ESP_LOGE(TAG, "  [4] SDA<->SDA, SCL<->SCL (verificar continuidade c/ multimetro)");
    ESP_LOGE(TAG, "  [5] Cabos curtos (<10 cm); jumpers firmes nas barras de pino");
    ESP_LOGE(TAG, "  [6] AD0 do MPU6050 NAO flutuante: GND -> 0x68, VCC -> 0x69");
    ESP_LOGE(TAG, "  [7] Scope: SCL deve clockar @ 400 kHz; SDA com ACK no 9o bit");
    ESP_LOGE(TAG, "  [8] Reduza I2C_FREQ_HZ p/ 100000 se cabo longo / pull-up fraco");
}

// -------------------------------------------------------------------------
void evaluate(const bool found[128])
{
    const bool prim = found[MPU_ADDR_PRIMARY];
    const bool alt  = found[MPU_ADDR_ALT_AD0];

    if (!prim && !alt) {
        log_troubleshooting();
        return;
    }

    const uint8_t addr = prim ? MPU_ADDR_PRIMARY : MPU_ADDR_ALT_AD0;
    const char*   mode = prim ? "PRIMARIO 0x68 (AD0=GND)" : "ALTERNATIVO 0x69 (AD0=VCC)";
    ESP_LOGI(TAG, "PASS: ACK em endereco %s", mode);

    uint8_t who = 0;
    esp_err_t err = read_register(addr, MPU_REG_WHO_AM_I, who);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read FAIL (%s) — bus instavel? releitura no proximo ciclo",
                 esp_err_to_name(err));
        return;
    }

    if (who == MPU_WHO_AM_I_OK) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02x — chip valido. Bring-up OK.", who);
    } else {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02x (esperado 0x%02x). Clone? Outro CI no mesmo addr?",
                 who, MPU_WHO_AM_I_OK);
    }
}

}  // namespace

// -------------------------------------------------------------------------
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "==== I2C SCANNER — Diagnostico Fase 0.1 ====");
    ESP_LOGI(TAG, "Bus I2C_NUM_%d  SDA=GPIO%d  SCL=GPIO%d  Clock=%" PRIu32 " Hz",
             I2C_PORT, SDA_PIN, SCL_PIN, I2C_FREQ_HZ);

    // Suprime warnings do driver durante probe (NACK em endereco vazio nao e erro).
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);

    esp_err_t err = bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus FAIL: %s", esp_err_to_name(err));
        return;
    }

    uint32_t cycle = 0;
    while (true) {
        ++cycle;
        ESP_LOGI(TAG, "--- Ciclo #%" PRIu32 " ---", cycle);

        bool found[128] = {};
        for (uint8_t addr = SCAN_LO; addr <= SCAN_HI; ++addr) {
            const esp_err_t e = i2c_master_probe(g_bus, addr, PROBE_TIMEOUT_MS);
            found[addr] = (e == ESP_OK);
        }

        print_detect_table(found);
        evaluate(found);

        vTaskDelay(pdMS_TO_TICKS(CYCLE_PERIOD_MS));
    }
}
