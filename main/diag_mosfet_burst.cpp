// =============================================================================
// diag_mosfet_burst.cpp — Firmware standalone Fase 0.2
//
// Caracteriza UM MOSFET (IRLML6344, M1 -> GPIO38) com perfil:
//   1. Soft-start linear 0 -> 50% em 800 ms
//   2. Hold 50%       por 3000 ms (~1.5 A)
//   3. Burst 100%     por  200 ms (~3 A — caracteriza Vds/Tj)
//   4. Ramp down 100->0 em 1000 ms
//   5. Idle           5000 ms (com medicao de recovery)
//
// Coleta a cada ciclo: V_idle, V_burst_min, sag_mV, t_recovery_ms.
// ADC1_CH0 (GPIO1) com divisor 100k:33k. ATEN_DB_2_5 cobre 0..1.5V.
//
// Hardware obrigatorio (CLAUDE.md §6.1, §10):
//   - 47 ohm em serie no GATE (limita di/dt e ringing)
//   - 10 k pull-down GATE->GND (boot-safe: motor parado antes do firmware ligar)
//   - SS14 freewheel diode (anodo no dreno, catodo no VBAT+)
//   - 1000 uF low-ESR no barramento, < 5 mm do dreno
//   - copper pour 200 mm^2 no dreno (RθJA ~80°C/W)
//
// "Codigo todo em main.cpp para facilitar bring-up" — sem componentes.
// =============================================================================

#include <cinttypes>
#include <cstdint>

#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "MOSFET-M1";

// ----- Hardware (espelha pinmap.hpp / CLAUDE.md §3.3) -----
constexpr int           MOTOR_GPIO  = 38;                 // M1 frente esquerda
constexpr adc_channel_t VBAT_CH     = ADC_CHANNEL_0;      // GPIO1
constexpr adc_unit_t    VBAT_UNIT   = ADC_UNIT_1;
constexpr adc_atten_t   VBAT_ATTEN  = ADC_ATTEN_DB_2_5;   // ~0..1.5V no pino

// Divisor de tensao 100k em serie + 33k para GND.
// V_pin = V_bat * 33 / (100+33) -> V_bat = V_pin * 133/33 = V_pin * 4.030
constexpr float VBAT_DIVIDER_RATIO = (100.0f + 33.0f) / 33.0f;

// ----- PWM (CLAUDE.md §10) -----
constexpr uint32_t PWM_RES_HZ      = 160000000u;          // MCPWM @ 160 MHz
constexpr uint32_t PWM_PERIOD_TICK = 8000u;               // 160e6 / 8000 = 20 kHz
constexpr uint32_t PWM_FREQ_HZ     = PWM_RES_HZ / PWM_PERIOD_TICK;
constexpr uint32_t DUTY_0          = 0u;
constexpr uint32_t DUTY_50         = PWM_PERIOD_TICK / 2u;  // 4000
constexpr uint32_t DUTY_100        = PWM_PERIOD_TICK;        // 8000

// ----- Sequencia -----
constexpr uint32_t RAMP_UP_MS            = 800u;
constexpr uint32_t HOLD_50_MS            = 3000u;
constexpr uint32_t BURST_100_MS          = 200u;
constexpr uint32_t RAMP_DOWN_MS          = 1000u;
constexpr uint32_t IDLE_MS               = 5000u;
constexpr uint32_t RECOVERY_TIMEOUT_MS   = 1500u;
constexpr float    RECOVERY_THRESHOLD_MV = 50.0f;

// ----- Watchdog -----
constexpr uint32_t TWDT_TIMEOUT_MS = 6000u;

// ----- Globais (singletons; firmware mono-threaded) -----
mcpwm_timer_handle_t      g_timer    = nullptr;
mcpwm_oper_handle_t       g_oper     = nullptr;
mcpwm_cmpr_handle_t       g_cmp      = nullptr;
mcpwm_gen_handle_t        g_gen      = nullptr;
adc_oneshot_unit_handle_t g_adc      = nullptr;
adc_cali_handle_t         g_adc_cali = nullptr;

// =========================================================================
// PWM
// =========================================================================
esp_err_t pwm_init()
{
    mcpwm_timer_config_t tcfg{};
    tcfg.group_id      = 0;
    tcfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tcfg.resolution_hz = PWM_RES_HZ;
    tcfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    tcfg.period_ticks  = PWM_PERIOD_TICK;
    if (esp_err_t e = mcpwm_new_timer(&tcfg, &g_timer); e != ESP_OK) return e;

    mcpwm_operator_config_t ocfg{};
    ocfg.group_id = 0;
    if (esp_err_t e = mcpwm_new_operator(&ocfg, &g_oper); e != ESP_OK) return e;
    if (esp_err_t e = mcpwm_operator_connect_timer(g_oper, g_timer); e != ESP_OK) return e;

    mcpwm_comparator_config_t ccfg{};
    ccfg.flags.update_cmp_on_tez = true;     // atualiza compare no reset do timer (sync)
    if (esp_err_t e = mcpwm_new_comparator(g_oper, &ccfg, &g_cmp); e != ESP_OK) return e;

    mcpwm_generator_config_t gcfg{};
    gcfg.gen_gpio_num = MOTOR_GPIO;
    if (esp_err_t e = mcpwm_new_generator(g_oper, &gcfg, &g_gen); e != ESP_OK) return e;

    // HIGH em TEZ (timer empty / inicio do periodo), LOW em compare match.
    mcpwm_gen_timer_event_action_t te_act{};
    te_act.direction = MCPWM_TIMER_DIRECTION_UP;
    te_act.event     = MCPWM_TIMER_EVENT_EMPTY;
    te_act.action    = MCPWM_GEN_ACTION_HIGH;
    if (esp_err_t e = mcpwm_generator_set_action_on_timer_event(g_gen, te_act); e != ESP_OK) return e;

    mcpwm_gen_compare_event_action_t cmp_act{};
    cmp_act.direction  = MCPWM_TIMER_DIRECTION_UP;
    cmp_act.comparator = g_cmp;
    cmp_act.action     = MCPWM_GEN_ACTION_LOW;
    if (esp_err_t e = mcpwm_generator_set_action_on_compare_event(g_gen, cmp_act); e != ESP_OK) return e;

    if (esp_err_t e = mcpwm_comparator_set_compare_value(g_cmp, 0); e != ESP_OK) return e;
    if (esp_err_t e = mcpwm_timer_enable(g_timer); e != ESP_OK) return e;
    return mcpwm_timer_start_stop(g_timer, MCPWM_TIMER_START_NO_STOP);
}

inline void set_duty(uint32_t duty_ticks)
{
    if (duty_ticks > PWM_PERIOD_TICK) duty_ticks = PWM_PERIOD_TICK;
    mcpwm_comparator_set_compare_value(g_cmp, static_cast<uint32_t>(duty_ticks));
}

// =========================================================================
// ADC
// =========================================================================
esp_err_t adc_init()
{
    adc_oneshot_unit_init_cfg_t ucfg{};
    ucfg.unit_id = VBAT_UNIT;
    if (esp_err_t e = adc_oneshot_new_unit(&ucfg, &g_adc); e != ESP_OK) return e;

    adc_oneshot_chan_cfg_t ccfg{};
    ccfg.atten    = VBAT_ATTEN;
    ccfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (esp_err_t e = adc_oneshot_config_channel(g_adc, VBAT_CH, &ccfg); e != ESP_OK) return e;

    adc_cali_curve_fitting_config_t calcfg{};
    calcfg.unit_id  = VBAT_UNIT;
    calcfg.chan     = VBAT_CH;
    calcfg.atten    = VBAT_ATTEN;
    calcfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    return adc_cali_create_scheme_curve_fitting(&calcfg, &g_adc_cali);
}

float read_vbat_mv()
{
    int raw = 0;
    if (adc_oneshot_read(g_adc, VBAT_CH, &raw) != ESP_OK) return 0.0f;
    int mv_pin = 0;
    if (adc_cali_raw_to_voltage(g_adc_cali, raw, &mv_pin) != ESP_OK) return 0.0f;
    return static_cast<float>(mv_pin) * VBAT_DIVIDER_RATIO;  // mV no barramento VBAT
}

float read_vbat_averaged(int n_samples)
{
    float sum = 0.0f;
    for (int i = 0; i < n_samples; ++i) {
        sum += read_vbat_mv();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return sum / static_cast<float>(n_samples);
}

// =========================================================================
// SEQUENCIA
// =========================================================================
struct CycleReport {
    uint32_t cycle_num;
    float    v_idle_mv;
    float    v_burst_min_mv;
    float    v_sag_mv;
    uint32_t t_recovery_ms;
    bool     recovery_ok;
};

void linear_ramp(uint32_t duty_start, uint32_t duty_end, uint32_t duration_ms)
{
    const int32_t s     = static_cast<int32_t>(duty_start);
    const int32_t e     = static_cast<int32_t>(duty_end);
    const int32_t delta = e - s;
    const int32_t dur   = static_cast<int32_t>(duration_ms);

    TickType_t last = xTaskGetTickCount();
    for (int32_t t = 0; t <= dur; ++t) {
        const int32_t d = s + (delta * t) / dur;
        set_duty(static_cast<uint32_t>(d));
        esp_task_wdt_reset();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}

void run_cycle(CycleReport& rep)
{
    // 1) Baseline V_idle (10 amostras, 1 ms cada — suaviza ruido de 50/60 Hz).
    rep.v_idle_mv      = read_vbat_averaged(10);
    rep.v_burst_min_mv = rep.v_idle_mv;  // sentinela: pior caso = V_idle (sem sag)

    // 2) Soft-start 0 -> 50%
    linear_ramp(DUTY_0, DUTY_50, RAMP_UP_MS);

    // 3) Hold 50%
    set_duty(DUTY_50);
    {
        const TickType_t hold_start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - hold_start) < pdMS_TO_TICKS(HOLD_50_MS)) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    // 4) BURST 100% — sample VBAT a 1 ms, captura V_burst_min
    set_duty(DUTY_100);
    {
        TickType_t last = xTaskGetTickCount();
        for (uint32_t t = 0; t < BURST_100_MS; ++t) {
            const float v = read_vbat_mv();
            if (v > 0.0f && v < rep.v_burst_min_mv) {
                rep.v_burst_min_mv = v;
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        }
    }
    rep.v_sag_mv = rep.v_idle_mv - rep.v_burst_min_mv;
    if (rep.v_sag_mv < 0.0f) rep.v_sag_mv = 0.0f;

    // 5) Ramp down 100 -> 0
    linear_ramp(DUTY_100, DUTY_0, RAMP_DOWN_MS);
    set_duty(DUTY_0);

    // 6) Recovery: tempo ate V >= V_idle - 50 mV
    rep.recovery_ok   = false;
    rep.t_recovery_ms = 0;
    const float threshold = rep.v_idle_mv - RECOVERY_THRESHOLD_MV;
    {
        const TickType_t rec_start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - rec_start) < pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS)) {
            const float v = read_vbat_mv();
            if (v >= threshold) {
                rep.t_recovery_ms = pdTICKS_TO_MS(xTaskGetTickCount() - rec_start);
                rep.recovery_ok   = true;
                break;
            }
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!rep.recovery_ok) rep.t_recovery_ms = RECOVERY_TIMEOUT_MS;
    }

    // 7) Idle remaining (fecha 5 s totais pos-rampa)
    const uint32_t idle_remain = (rep.t_recovery_ms < IDLE_MS)
        ? (IDLE_MS - rep.t_recovery_ms) : 0u;
    if (idle_remain > 0u) {
        const TickType_t idle_start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - idle_start) < pdMS_TO_TICKS(idle_remain)) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void log_report(const CycleReport& r)
{
    ESP_LOGI(TAG,
             "Ciclo #%" PRIu32 " | V_idle=%4d mV | V_burst_min=%4d mV | sag=%4d mV | t_recovery=%" PRIu32 " ms %s",
             r.cycle_num,
             static_cast<int>(r.v_idle_mv),
             static_cast<int>(r.v_burst_min_mv),
             static_cast<int>(r.v_sag_mv),
             r.t_recovery_ms,
             r.recovery_ok ? "OK" : "TIMEOUT");

    // Avaliacao automatica de criterios (CLAUDE.md §6.1)
    if (r.v_sag_mv > 200.0f) {
        ESP_LOGW(TAG, "  -> SAG > 200 mV: bulk cap subdimensionado / longe / ESR de bateria alta.");
    }
    if (!r.recovery_ok) {
        ESP_LOGE(TAG, "  -> RECOVERY TIMEOUT (%" PRIu32 " ms): bateria fraca, brownout iminente.",
                 RECOVERY_TIMEOUT_MS);
    }
    if (r.v_idle_mv < 3500.0f) {
        ESP_LOGW(TAG, "  -> V_idle = %d mV < 3500 mV: terminar testes e recarregar bateria.",
                 static_cast<int>(r.v_idle_mv));
    }
}

}  // namespace

// =========================================================================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "==== MOSFET BURST CHARACTERIZATION — Diag Fase 0.2 ====");
    ESP_LOGI(TAG, "Motor M1 -> GPIO%d  |  PWM %" PRIu32 " Hz  |  Period=%" PRIu32 " ticks",
             MOTOR_GPIO, PWM_FREQ_HZ, PWM_PERIOD_TICK);
    ESP_LOGW(TAG, "GATE: 47 ohm serie OBRIGATORIO; pull-down 10k GATE->GND.");
    ESP_LOGW(TAG, "FREEWHEEL: SS14 (anodo->dreno, catodo->VBAT+) — sem ele MOSFET morre no turn-off.");
    ESP_LOGW(TAG, "BULK: 1000 uF low-ESR a < 5 mm do dreno (CLAUDE.md §6.1).");

    // TWDT 6 s (overrides default de 5 s do sdkconfig.defaults)
    esp_task_wdt_config_t wdt_cfg{};
    wdt_cfg.timeout_ms     = TWDT_TIMEOUT_MS;
    wdt_cfg.idle_core_mask = (1u << 0) | (1u << 1);
    wdt_cfg.trigger_panic  = true;
    if (esp_err_t e = esp_task_wdt_reconfigure(&wdt_cfg); e != ESP_OK) {
        ESP_LOGW(TAG, "TWDT reconfigure: %s — segue com timeout default.", esp_err_to_name(e));
    }
    if (esp_err_t e = esp_task_wdt_add(nullptr); e != ESP_OK) {
        ESP_LOGW(TAG, "TWDT add task: %s", esp_err_to_name(e));
    }

    if (esp_err_t e = pwm_init(); e != ESP_OK) {
        ESP_LOGE(TAG, "pwm_init FAIL: %s", esp_err_to_name(e));
        return;
    }
    if (esp_err_t e = adc_init(); e != ESP_OK) {
        ESP_LOGE(TAG, "adc_init FAIL: %s", esp_err_to_name(e));
        return;
    }
    set_duty(0);  // motor parado antes de qualquer aquisicao

    ESP_LOGI(TAG, "Setup OK. Iniciando ciclos em 3 s — verifique probes do scope.");
    for (int i = 0; i < 30; ++i) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    uint32_t cycle = 0;
    while (true) {
        ++cycle;
        CycleReport r{};
        r.cycle_num = cycle;
        run_cycle(r);
        log_report(r);
        esp_task_wdt_reset();
    }
}
