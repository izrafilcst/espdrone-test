// =============================================================================
// diag_camera_frame.cpp — Firmware standalone Fase 0.4
//
// Valida pipeline OV2640 → DVP 8-bit → JPEG hardware → PSRAM (CLAUDE.md §3.3):
//   XCLK=GPIO15  SIOD=GPIO4  SIOC=GPIO5
//   VSYNC=GPIO6  HREF=GPIO7  PCLK=GPIO13
//   Y2..Y9 = GPIO11, 9, 8, 10, 12, 18, 17, 16
//
// Config: PIXFORMAT_JPEG, FRAMESIZE_QVGA (320×240), quality=12,
//         fb_count=2 em PSRAM, grab_mode=LATEST.
//
// Loop:
//   1. esp_camera_fb_get()
//   2. Valida: fb!=nullptr, len>=2048, SOI=0xFF 0xD8
//   3. Log "[CAM] frame=N len=BYTES width=W height=H elapsed_ms=T"
//   4. A cada 10 frames: hex dump dos primeiros 64 bytes (header JPEG)
//   5. esp_camera_fb_return()
//   6. FPS medio em janelas de 30 frames
//
// Se fb==nullptr 3x consecutivas: diagnostico de hardware detalhado.
// Sem Wi-Fi — teste puro do front-end de imagem.
//
// "Codigo todo em main/ para facilitar bring-up" — sem componentes.
// =============================================================================

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "CAM";

// ----- Hardware (espelha pinmap.hpp / CLAUDE.md §3.3) -----
constexpr int CAM_PIN_PWDN   = -1;   // NC — sem pino de power-down
constexpr int CAM_PIN_RESET  = -1;   // NC — reset por software
constexpr int CAM_PIN_XCLK   = 15;
constexpr int CAM_PIN_SIOD   =  4;   // SCCB SDA
constexpr int CAM_PIN_SIOC   =  5;   // SCCB SCL
constexpr int CAM_PIN_VSYNC  =  6;
constexpr int CAM_PIN_HREF   =  7;
constexpr int CAM_PIN_PCLK   = 13;
constexpr int CAM_PIN_Y2     = 11;   // D0 (LSB)
constexpr int CAM_PIN_Y3     =  9;   // D1
constexpr int CAM_PIN_Y4     =  8;   // D2
constexpr int CAM_PIN_Y5     = 10;   // D3
constexpr int CAM_PIN_Y6     = 12;   // D4
constexpr int CAM_PIN_Y7     = 18;   // D5
constexpr int CAM_PIN_Y8     = 17;   // D6
constexpr int CAM_PIN_Y9     = 16;   // D7 (MSB)

// ----- Parametros de captura -----
constexpr uint32_t XCLK_FREQ_HZ    = 20000000U;  // 20 MHz (LEDC)
constexpr int      JPEG_QUALITY    = 12;
constexpr size_t   MIN_FRAME_BYTES = 2048;        // criterio: nenhum frame < 2 KB
constexpr uint32_t FPS_WINDOW      = 30;          // media a cada 30 frames
constexpr uint32_t NULL_FAIL_LIMIT = 3;           // fb==nullptr consecutivos antes do diag

// ----- Criterio de sucesso (sustentacao) -----
constexpr int64_t SUSTAIN_US      = 60LL * 1000000LL;  // 60 s em microsegundos
constexpr float   SUSTAIN_FPS_MIN = 25.0f;              // >= 25 FPS por 60 s

// =========================================================================
// Diagnostico de hardware — @hardware-debugging
// Acionado apos NULL_FAIL_LIMIT capturas nulas consecutivas.
// =========================================================================
void print_hw_diag()
{
    ESP_LOGE(TAG, "====== DIAGNOSTICO DE HARDWARE ======");
    ESP_LOGE(TAG, "fb==nullptr por %" PRIu32 " capturas consecutivas.", NULL_FAIL_LIMIT);
    ESP_LOGE(TAG, "Verificar:");
    ESP_LOGE(TAG, "  1. Alimentacao 3.3V no modulo OV2640");
    ESP_LOGE(TAG, "     XCLK_PIN=GPIO%d (LEDC_TIMER_0 @ 20 MHz) — medir com osciloscopio",
             CAM_PIN_XCLK);
    ESP_LOGE(TAG, "  2. Pull-ups SCCB: SIOD=GPIO%d e SIOC=GPIO%d",
             CAM_PIN_SIOD, CAM_PIN_SIOC);
    ESP_LOGE(TAG, "     Requer 10 kOhm para 3.3V (pull-ups dedicados, nao os do IMU)");
    ESP_LOGE(TAG, "  3. Aterramento: GND do OV2640 compartilhado com ESP32-S3");
    ESP_LOGE(TAG, "  4. Antenas paralelas Y2..Y9 (barramento DVP 8-bit):");
    ESP_LOGE(TAG, "     Y2=GPIO%-2d  Y3=GPIO%-2d  Y4=GPIO%-2d  Y5=GPIO%-2d",
             CAM_PIN_Y2, CAM_PIN_Y3, CAM_PIN_Y4, CAM_PIN_Y5);
    ESP_LOGE(TAG, "     Y6=GPIO%-2d  Y7=GPIO%-2d  Y8=GPIO%-2d  Y9=GPIO%-2d",
             CAM_PIN_Y6, CAM_PIN_Y7, CAM_PIN_Y8, CAM_PIN_Y9);
    ESP_LOGE(TAG, "     Verificar continuidade com multimetro (modo bip)");
    ESP_LOGE(TAG, "  5. Sincronismo DVP:");
    ESP_LOGE(TAG, "     VSYNC=GPIO%d (~30 Hz)  HREF=GPIO%d  PCLK=GPIO%d (~6 MHz QVGA)",
             CAM_PIN_VSYNC, CAM_PIN_HREF, CAM_PIN_PCLK);
    ESP_LOGE(TAG, "  6. PWDN e RESET sao NC — confirmar que nao estao aterrados");
    ESP_LOGE(TAG, "=====================================");
}

// =========================================================================
// Hex dump — primeiros max_bytes do buffer, 16 por linha
// =========================================================================
void hex_dump(const uint8_t* buf, size_t len, size_t max_bytes)
{
    if (len < max_bytes) max_bytes = len;
    for (size_t i = 0; i < max_bytes; i += 16) {
        char line[64] = {};
        size_t lp = 0;
        for (size_t j = i; j < i + 16 && j < max_bytes; ++j) {
            lp += static_cast<size_t>(
                std::snprintf(line + lp, sizeof(line) - lp, "%02X ", buf[j]));
        }
        ESP_LOGI(TAG, "  [+%03zu] %s", i, line);
    }
}

// =========================================================================
// Inicializacao da camera
// =========================================================================
esp_err_t camera_init()
{
    esp_camera_config_t cfg{};
    cfg.pin_pwdn      = CAM_PIN_PWDN;
    cfg.pin_reset     = CAM_PIN_RESET;
    cfg.pin_xclk      = CAM_PIN_XCLK;
    cfg.pin_sccb_sda  = CAM_PIN_SIOD;
    cfg.pin_sccb_scl  = CAM_PIN_SIOC;
    cfg.pin_vsync     = CAM_PIN_VSYNC;
    cfg.pin_href      = CAM_PIN_HREF;
    cfg.pin_pclk      = CAM_PIN_PCLK;
    cfg.pin_d0        = CAM_PIN_Y2;
    cfg.pin_d1        = CAM_PIN_Y3;
    cfg.pin_d2        = CAM_PIN_Y4;
    cfg.pin_d3        = CAM_PIN_Y5;
    cfg.pin_d4        = CAM_PIN_Y6;
    cfg.pin_d5        = CAM_PIN_Y7;
    cfg.pin_d6        = CAM_PIN_Y8;
    cfg.pin_d7        = CAM_PIN_Y9;
    cfg.xclk_freq_hz  = XCLK_FREQ_HZ;
    cfg.ledc_timer    = LEDC_TIMER_0;
    cfg.ledc_channel  = LEDC_CHANNEL_0;
    cfg.pixel_format  = PIXFORMAT_JPEG;
    cfg.frame_size    = FRAMESIZE_QVGA;   // 320 x 240
    cfg.jpeg_quality  = JPEG_QUALITY;     // 10..15 — menor = melhor qualidade
    cfg.fb_count      = 2;                // double-buffer em PSRAM
    cfg.fb_location   = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode     = CAMERA_GRAB_LATEST;
    return esp_camera_init(&cfg);
}

// =========================================================================
// Task de captura — Core 0, prio 10 (CLAUDE.md §4.2 task_camera_stream)
// =========================================================================
void capture_task(void*)
{
    uint32_t frame_count      = 0;
    uint32_t null_streak      = 0;
    int64_t  fps_window_start = esp_timer_get_time();

    // Janela de sustentacao: reset em qualquer frame invalido ou nullptr
    struct {
        int64_t  start_us = 0;
        uint32_t frames   = 0;
        bool     active   = false;
    } sustain{};

    while (true) {
        const int64_t t0 = esp_timer_get_time();
        camera_fb_t*  fb = esp_camera_fb_get();
        const int64_t t1 = esp_timer_get_time();

        // ---- fb==nullptr: incrementa streak, reseta sustain, dispara diag ----
        if (fb == nullptr) {
            ++null_streak;
            ESP_LOGW(TAG, "esp_camera_fb_get() retornou nullptr (streak=%" PRIu32 ")",
                     null_streak);
            if (sustain.active) {
                ESP_LOGW(TAG,
                         "[SUSTAIN] janela resetada — fb=nullptr apos %" PRIu32 " frames validos",
                         sustain.frames);
                sustain.active = false;
                sustain.frames = 0;
            }
            if (null_streak >= NULL_FAIL_LIMIT) {
                print_hw_diag();
                null_streak = 0;
                vTaskDelay(pdMS_TO_TICKS(5000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }
        null_streak = 0;
        ++frame_count;

        const uint32_t elapsed_ms = static_cast<uint32_t>((t1 - t0) / 1000);

        // ---- Validacoes de sanidade ----
        bool        valid       = true;
        const char* fail_reason = nullptr;

        if (fb->len < MIN_FRAME_BYTES) {
            ESP_LOGW(TAG, "frame %" PRIu32 ": len=%zu < %zu (abaixo de 2 KB)",
                     frame_count, fb->len, MIN_FRAME_BYTES);
            valid       = false;
            fail_reason = "len<2KB";
        }

        if (fb->len >= 2 && (fb->buf[0] != 0xFF || fb->buf[1] != 0xD8)) {
            ESP_LOGW(TAG,
                     "frame %" PRIu32 ": SOI incorreto [%02X %02X] — esperado FF D8 (JPEG)",
                     frame_count, fb->buf[0], fb->buf[1]);
            valid = false;
            if (fail_reason == nullptr) fail_reason = "bad-SOI";
        }

        // ---- Janela de sustentacao (criterio de sucesso da Fase 0.4) ----
        if (!valid) {
            if (sustain.active) {
                ESP_LOGW(TAG,
                         "[SUSTAIN] janela resetada — frame invalido (%s) apos %" PRIu32 " frames",
                         fail_reason ? fail_reason : "?", sustain.frames);
                sustain.active = false;
                sustain.frames = 0;
            }
        } else {
            if (!sustain.active) {
                sustain.active   = true;
                sustain.start_us = t1;
                sustain.frames   = 0;
                ESP_LOGI(TAG, "[SUSTAIN] janela iniciada — aguardando 60 s a >= %.0f FPS",
                         SUSTAIN_FPS_MIN);
            }
            ++sustain.frames;

            const int64_t elapsed_sustain = t1 - sustain.start_us;
            if (elapsed_sustain >= SUSTAIN_US) {
                const float fps_s = static_cast<float>(sustain.frames) * 1000000.0f
                                    / static_cast<float>(elapsed_sustain);
                if (fps_s >= SUSTAIN_FPS_MIN) {
                    ESP_LOGI(TAG,
                             "[SUSTAIN] *** PASS *** %" PRIu32 " frames em %.1f s = %.1f FPS"
                             " (alvo: %.0f FPS, todos FF D8, todos >= 2 KB)",
                             sustain.frames,
                             static_cast<float>(elapsed_sustain) / 1000000.0f,
                             fps_s, SUSTAIN_FPS_MIN);
                } else {
                    ESP_LOGW(TAG,
                             "[SUSTAIN] FAIL FPS — %.1f FPS < %.0f FPS alvo"
                             " (%" PRIu32 " frames em 60 s)",
                             fps_s, SUSTAIN_FPS_MIN, sustain.frames);
                }
                sustain.active = false;  // reset para monitoramento continuo
                sustain.frames = 0;
            }
        }

        // ---- Log principal ----
        ESP_LOGI(TAG,
                 "[CAM] frame=%" PRIu32 " len=%zu width=%" PRIu32
                 " height=%" PRIu32 " elapsed_ms=%" PRIu32 "  %s",
                 frame_count,
                 fb->len,
                 static_cast<uint32_t>(fb->width),
                 static_cast<uint32_t>(fb->height),
                 elapsed_ms,
                 valid ? "OK" : "WARN");

        // ---- Dump hex a cada 10 frames ----
        if (frame_count % 10 == 0) {
            ESP_LOGI(TAG, "--- hex dump frame %" PRIu32 " (primeiros 64 bytes) ---",
                     frame_count);
            hex_dump(fb->buf, fb->len, 64);
        }

        esp_camera_fb_return(fb);

        // ---- FPS medio em janela de FPS_WINDOW frames ----
        if (frame_count % FPS_WINDOW == 0) {
            const int64_t now_us    = esp_timer_get_time();
            const int64_t window_us = now_us - fps_window_start;
            const float   fps = static_cast<float>(FPS_WINDOW) * 1000000.0f
                                / static_cast<float>(window_us);
            ESP_LOGI(TAG, "FPS (ultimos %" PRIu32 " frames): %.1f", FPS_WINDOW, fps);
            fps_window_start = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // yield — nao monopoliza Core 0
    }
}

}  // namespace

// =========================================================================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "==== OV2640 CAMERA DIAG — Fase 0.4 ====");
    ESP_LOGI(TAG, "Pinagem: XCLK=%d  SIOD=%d  SIOC=%d  VSYNC=%d  HREF=%d  PCLK=%d",
             CAM_PIN_XCLK, CAM_PIN_SIOD, CAM_PIN_SIOC,
             CAM_PIN_VSYNC, CAM_PIN_HREF, CAM_PIN_PCLK);
    ESP_LOGI(TAG, "DVP Y2-Y9: %d %d %d %d %d %d %d %d",
             CAM_PIN_Y2, CAM_PIN_Y3, CAM_PIN_Y4, CAM_PIN_Y5,
             CAM_PIN_Y6, CAM_PIN_Y7, CAM_PIN_Y8, CAM_PIN_Y9);
    ESP_LOGI(TAG, "Config: JPEG QVGA quality=%d  fb=2  PSRAM  grab=LATEST",
             JPEG_QUALITY);

    const esp_err_t err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init FAIL: %s (0x%x)", esp_err_to_name(err), err);
        ESP_LOGE(TAG, "Causas comuns:");
        ESP_LOGE(TAG, "  - PSRAM Octal ausente: sdkconfig CONFIG_SPIRAM_MODE_OCT=y?");
        ESP_LOGE(TAG, "  - OV2640 nao detectado via SCCB (probe 0x30/0x31)");
        ESP_LOGE(TAG, "  - XCLK sem sinal: LEDC_TIMER_0 em GPIO%d", CAM_PIN_XCLK);
        ESP_LOGE(TAG, "  - Pull-ups SCCB ausentes (10 kOhm para 3.3V)");
        return;
    }

    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor != nullptr) {
        ESP_LOGI(TAG, "Sensor detectado: PID=0x%04X  SCCB=0x%02X",
                 sensor->id.PID, sensor->slv_addr);
    }

    ESP_LOGI(TAG, "Camera OK. Iniciando captura…");
    xTaskCreatePinnedToCore(capture_task, "cam_capture", 8192, nullptr, 10, nullptr, 0);
}
