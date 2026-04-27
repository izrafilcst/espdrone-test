// =============================================================================
// diag_gps_nmea.cpp — Firmware standalone Fase 0.3
//
// Valida bring-up do GPS NEO-6M via UART2 (CLAUDE.md §3.3):
//   - RX=GPIO14, TX=GPIO42, 9600-8N1
//   - Parser inline de $..RMC e $..GGA com checksum XOR
//   - Resumo a 1 Hz, CLI "gps stats" + "gps last_rmc"
//
// Self-tests com sentencas hardcoded rodam ANTES de habilitar a UART RX:
//   T1) RMC valida (checksum publico do datasheet) -> ACCEPT
//   T2) Checksum corrompido                        -> REJECT
//   T3) Sentenca truncada (sem '*')                -> REJECT
//   T4) RMC com lat 0000.00 (sem fix)              -> ACCEPT + flag no_fix
//
// Falha de qualquer self-test -> abort() (parser quebrado nao vale dado real).
//
// "Codigo todo em main.cpp para facilitar bring-up" — sem componentes.
// =============================================================================

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "GPS";

// ----- Hardware (espelha pinmap.hpp / CLAUDE.md §3.3) -----
constexpr uart_port_t GPS_UART_PORT = UART_NUM_2;
constexpr int         GPS_RX_GPIO   = 14;
constexpr int         GPS_TX_GPIO   = 42;
constexpr int         GPS_BAUDRATE  = 9600;
constexpr size_t      UART_RX_BUF   = 1024;  // "buffer circular de 1024 bytes"

// ----- Parser -----
constexpr size_t LINE_MAX = 96;   // NMEA limit oficial: 82 chars + folga
constexpr size_t RMC_RAW  = 96;

// =========================================================================
// Estado compartilhado
// =========================================================================
struct GpsSnapshot {
    char utc[12];
    char date[8];
    char status;        // 'A' (active) | 'V' (void) | '?' (sem RMC ainda)
    char lat[14];       // raw NMEA: "DDMM.mmmm"
    char ns;            // 'N' | 'S'
    char lon[14];       // raw NMEA: "DDDMM.mmmm"
    char ew;            // 'E' | 'W'
    char speed[8];      // RMC field 7 — knots, ex: "022.4"
    char course[8];     // RMC field 8 — degrees true, ex: "084.4"
    char alt[10];
    int  num_sats;
    int  fix_quality;   // 0 (no fix) | 1 (GPS) | 2 (DGPS)
    bool no_fix;
};

struct GpsStats {
    uint32_t total;
    uint32_t valid;
    uint32_t rejected;
};

GpsSnapshot      g_snap{};
GpsStats         g_stats{};
char             g_last_rmc[RMC_RAW] = {0};
SemaphoreHandle_t g_mtx = nullptr;
int64_t          g_boot_us = 0;

// Acumulador de linha (rodado pela RX task; nao precisa de lock — single-writer)
char   g_line[LINE_MAX];
size_t g_line_len = 0;

// =========================================================================
// Helpers de lock (timeout 100 ms — CLAUDE.md §5.1 espirito)
// =========================================================================
bool lock_take()  { return xSemaphoreTake(g_mtx, pdMS_TO_TICKS(100)) == pdTRUE; }
void lock_give()  { xSemaphoreGive(g_mtx); }

// =========================================================================
// Validacao de checksum XOR
//   Linha esperada: "$XXXXX,...,Y*HH"  (sem \r\n)
// =========================================================================
int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool validate_checksum(const char* line, size_t len)
{
    if (len < 4 || line[0] != '$') return false;

    // localiza '*'
    size_t star = 0;
    bool found = false;
    for (size_t i = 1; i < len; ++i) {
        if (line[i] == '*') { star = i; found = true; break; }
    }
    if (!found) return false;
    if (star + 2 >= len) return false;  // precisa de 2 hex chars

    uint8_t xor_calc = 0;
    for (size_t i = 1; i < star; ++i) {
        xor_calc ^= static_cast<uint8_t>(line[i]);
    }

    const int hi = hex_digit(line[star + 1]);
    const int lo = hex_digit(line[star + 2]);
    if (hi < 0 || lo < 0) return false;

    return xor_calc == static_cast<uint8_t>((hi << 4) | lo);
}

// =========================================================================
// Iterador de campos (separados por ',' ou '*')
// =========================================================================
class FieldIter {
public:
    FieldIter(const char* s, size_t len) : cur_(s), end_(s + len) {}

    bool next(const char*& p, size_t& flen)
    {
        if (cur_ > end_) return false;
        const char* start = cur_;
        while (cur_ < end_ && *cur_ != ',' && *cur_ != '*') ++cur_;
        p = start;
        flen = static_cast<size_t>(cur_ - start);
        if (cur_ < end_ && (*cur_ == ',' || *cur_ == '*')) ++cur_;
        return true;
    }

    void skip_n(int n) { for (int i = 0; i < n; ++i) { const char* p; size_t l; next(p, l); } }

private:
    const char* cur_;
    const char* end_;
};

// Copia segura: campo (p,len) -> dst[dst_size], terminada em '\0'
void field_copy(char* dst, size_t dst_size, const char* p, size_t len)
{
    if (dst_size == 0) return;
    if (len >= dst_size) len = dst_size - 1;
    if (len > 0 && p) std::memcpy(dst, p, len);
    dst[len] = '\0';
}

// "0000.0000" / "0.0" / vazio -> tratado como sem fix
bool field_is_zero_coord(const char* p, size_t len)
{
    if (len == 0) return true;
    bool any_nonzero = false;
    for (size_t i = 0; i < len; ++i) {
        if (p[i] >= '1' && p[i] <= '9') { any_nonzero = true; break; }
    }
    return !any_nonzero;
}

// =========================================================================
// Talker / message-id matcher
//   Aceita $GPRMC, $GNRMC, $GLRMC, $GARMC etc.
//   Retorna true se line[3..6) == msg_id (3 chars).
// =========================================================================
bool is_message(const char* line, size_t len, const char* msg_id /*RMC|GGA*/)
{
    if (len < 6) return false;
    if (line[0] != '$') return false;
    return line[3] == msg_id[0] && line[4] == msg_id[1] && line[5] == msg_id[2];
}

// =========================================================================
// Parsers
// =========================================================================
// $..RMC,utc,status,lat,N/S,lon,E/W,speed,course,date,magvar,magvar_dir*CS
void parse_rmc(const char* line, size_t len)
{
    FieldIter it(line, len);
    const char* p; size_t l;
    it.next(p, l);  // descarta talker+msg ($..RMC)

    char utc[12]    = {0};
    char status     = '?';
    char lat[14]    = {0};
    char ns         = '?';
    char lon[14]    = {0};
    char ew         = '?';
    char speed[8]   = {0};
    char course[8]  = {0};
    char date[8]    = {0};
    bool no_fix     = false;

    if (it.next(p, l)) field_copy(utc, sizeof(utc), p, l);
    if (it.next(p, l) && l > 0) status = p[0];

    if (it.next(p, l)) {
        field_copy(lat, sizeof(lat), p, l);
        if (field_is_zero_coord(p, l)) no_fix = true;
    }
    if (it.next(p, l) && l > 0) ns = p[0];

    if (it.next(p, l)) {
        field_copy(lon, sizeof(lon), p, l);
        if (field_is_zero_coord(p, l)) no_fix = true;
    }
    if (it.next(p, l) && l > 0) ew = p[0];

    if (it.next(p, l)) field_copy(speed,  sizeof(speed),  p, l);
    if (it.next(p, l)) field_copy(course, sizeof(course), p, l);
    if (it.next(p, l)) field_copy(date,   sizeof(date),   p, l);

    // Status 'V' = void (no_fix forcado mesmo se lat/lon nao-zero)
    if (status == 'V') no_fix = true;

    if (lock_take()) {
        std::memcpy(g_snap.utc,    utc,    sizeof(utc));
        std::memcpy(g_snap.date,   date,   sizeof(date));
        std::memcpy(g_snap.lat,    lat,    sizeof(lat));
        std::memcpy(g_snap.lon,    lon,    sizeof(lon));
        std::memcpy(g_snap.speed,  speed,  sizeof(speed));
        std::memcpy(g_snap.course, course, sizeof(course));
        g_snap.status = status;
        g_snap.ns     = ns;
        g_snap.ew     = ew;
        g_snap.no_fix = no_fix;

        const size_t copy_len = (len < sizeof(g_last_rmc) - 1) ? len : (sizeof(g_last_rmc) - 1);
        std::memcpy(g_last_rmc, line, copy_len);
        g_last_rmc[copy_len] = '\0';
        lock_give();
    }
}

// $..GGA,utc,lat,N/S,lon,E/W,fix_qual,sats,hdop,alt,M,...
void parse_gga(const char* line, size_t len)
{
    FieldIter it(line, len);
    const char* p; size_t l;
    it.next(p, l);  // talker+msg

    it.skip_n(5);  // utc, lat, ns, lon, ew

    int  fix  = 0;
    int  sats = 0;
    char alt[10] = {0};

    if (it.next(p, l)) {
        char buf[4] = {0};
        field_copy(buf, sizeof(buf), p, l);
        fix = std::atoi(buf);
    }
    if (it.next(p, l)) {
        char buf[4] = {0};
        field_copy(buf, sizeof(buf), p, l);
        sats = std::atoi(buf);
    }
    it.skip_n(1);  // hdop
    if (it.next(p, l)) field_copy(alt, sizeof(alt), p, l);

    if (lock_take()) {
        g_snap.fix_quality = fix;
        g_snap.num_sats    = sats;
        std::memcpy(g_snap.alt, alt, sizeof(alt));
        lock_give();
    }
}

// =========================================================================
// Dispatcher
// =========================================================================
void process_line(const char* line, size_t len)
{
    if (lock_take()) { g_stats.total++; lock_give(); }

    if (!validate_checksum(line, len)) {
        if (lock_take()) { g_stats.rejected++; lock_give(); }
        return;
    }

    if (is_message(line, len, "RMC")) {
        parse_rmc(line, len);
    } else if (is_message(line, len, "GGA")) {
        parse_gga(line, len);
    }
    // Outras sentencas (VTG, GSA, GSV) sao validas mas ignoradas — contam como valid.

    if (lock_take()) { g_stats.valid++; lock_give(); }
}

// =========================================================================
// RX byte feeder
// =========================================================================
void feed_byte(char c)
{
    if (c == '$') {
        g_line_len = 0;
        g_line[g_line_len++] = c;
        return;
    }
    if (g_line_len == 0) return;

    if (c == '\r' || c == '\n') {
        if (g_line_len >= 6) {  // minimo: "$XX*HH"
            process_line(g_line, g_line_len);
        }
        g_line_len = 0;
        return;
    }
    if (g_line_len < LINE_MAX - 1) {
        g_line[g_line_len++] = c;
    } else {
        // overflow — descarta linha em curso (NMEA bem-formado nunca passa de 82)
        g_line_len = 0;
    }
}

// =========================================================================
// UART setup + RX task
// =========================================================================
esp_err_t uart_init()
{
    uart_config_t cfg{};
    cfg.baud_rate  = GPS_BAUDRATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t e;
    e = uart_driver_install(GPS_UART_PORT, UART_RX_BUF, 0, 0, nullptr, 0);
    if (e != ESP_OK) return e;
    e = uart_param_config(GPS_UART_PORT, &cfg);
    if (e != ESP_OK) return e;
    return uart_set_pin(GPS_UART_PORT, GPS_TX_GPIO, GPS_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void rx_task(void*)
{
    uint8_t tmp[128];
    while (true) {
        const int n = uart_read_bytes(GPS_UART_PORT, tmp, sizeof(tmp), pdMS_TO_TICKS(100));
        if (n <= 0) continue;
        for (int i = 0; i < n; ++i) {
            feed_byte(static_cast<char>(tmp[i]));
        }
    }
}

// =========================================================================
// Reporter task (1 Hz)
// =========================================================================
void reporter_task(void*)
{
    TickType_t last = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));

        GpsSnapshot s{};
        if (lock_take()) { s = g_snap; lock_give(); }

        ESP_LOGI(TAG, "[GPS] sats=%d fix=%d lat=%s%c lon=%s%c alt=%sm  %s",
                 s.num_sats, s.fix_quality,
                 s.lat[0] ? s.lat : "----", s.ns,
                 s.lon[0] ? s.lon : "-----", s.ew,
                 s.alt[0] ? s.alt : "--.-",
                 s.no_fix ? "[NO_FIX]" : "[FIX_OK]");
    }
}

// =========================================================================
// CLI: "gps stats" e "gps last_rmc"
// =========================================================================
int cmd_gps(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: gps {stats|last_rmc}\n");
        return 1;
    }

    if (std::strcmp(argv[1], "stats") == 0) {
        GpsStats st{};
        if (lock_take()) { st = g_stats; lock_give(); }
        const uint32_t up_s = static_cast<uint32_t>((esp_timer_get_time() - g_boot_us) / 1000000);
        std::printf("uptime=%" PRIu32 "s total=%" PRIu32 " valid=%" PRIu32 " rejected=%" PRIu32 "\n",
                    up_s, st.total, st.valid, st.rejected);
        return 0;
    }

    if (std::strcmp(argv[1], "last_rmc") == 0) {
        char buf[RMC_RAW] = {0};
        if (lock_take()) { std::memcpy(buf, g_last_rmc, sizeof(buf)); lock_give(); }
        std::printf("%s\n", buf[0] ? buf : "<nenhuma RMC valida ainda>");
        return 0;
    }

    std::printf("subcomando desconhecido: %s\n", argv[1]);
    return 1;
}

void register_cli()
{
    esp_console_cmd_t cmd{};
    cmd.command = "gps";
    cmd.help    = "GPS diag: gps stats | gps last_rmc";
    cmd.func    = &cmd_gps;
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// =========================================================================
// Self-tests (TDD: cada caso afirma um invariante observavel)
//   T1) RMC valida -> checksum aceito + TODOS campos extraidos corretos
//   T2) RMC com checksum corrompido -> REJECT
//   T3) RMC truncada (sem '*') -> REJECT
//   T4) RMC sintaticamente valida com lat/lon zerados -> no_fix=true
// Falha em qualquer caso -> abort() (parser quebrado nao vale dado real).
// =========================================================================
void run_self_tests()
{
    int failed = 0;

    // ---- T1: RMC valida + extracao completa --------------------------
    // Checksum 6A confere para "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"
    {
        const char* t1 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        const size_t t1_len = std::strlen(t1);

        if (!validate_checksum(t1, t1_len)) {
            ESP_LOGE(TAG, "T1 FAIL: validate_checksum rejeitou sentenca canonica");
            ++failed;
        } else {
            if (lock_take()) { g_snap = GpsSnapshot{}; lock_give(); }
            parse_rmc(t1, t1_len);

            GpsSnapshot s{};
            if (lock_take()) { s = g_snap; lock_give(); }

            int sub = 0;
            auto check_str = [&sub](const char* d, const char* got, const char* want) {
                if (std::strcmp(got, want) != 0) {
                    ESP_LOGE(TAG, "  T1.%s FAIL got='%s' want='%s'", d, got, want);
                    ++sub;
                }
            };
            auto check_char = [&sub](const char* d, char got, char want) {
                if (got != want) {
                    ESP_LOGE(TAG, "  T1.%s FAIL got='%c'(0x%02x) want='%c'", d,
                             got ? got : ' ', static_cast<unsigned>(got), want);
                    ++sub;
                }
            };

            check_str ("utc",    s.utc,    "123519");
            check_char("status", s.status, 'A');
            check_str ("lat",    s.lat,    "4807.038");
            check_char("ns",     s.ns,     'N');
            check_str ("lon",    s.lon,    "01131.000");
            check_char("ew",     s.ew,     'E');
            check_str ("speed",  s.speed,  "022.4");
            check_str ("course", s.course, "084.4");
            check_str ("date",   s.date,   "230394");
            if (s.no_fix) {
                ESP_LOGE(TAG, "  T1.no_fix FAIL: deveria ser false (status=A, lat/lon nao-zero)");
                ++sub;
            }

            if (sub == 0) {
                ESP_LOGI(TAG, "T1 RMC valida + extracao completa: PASS (10/10 campos)");
            } else {
                ESP_LOGE(TAG, "T1 FAIL: %d sub-falhas", sub);
                failed += sub;
            }
        }
    }

    // ---- T2: checksum corrompido -------------------------------------
    {
        const char* t2 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*FF";
        if (validate_checksum(t2, std::strlen(t2))) {
            ESP_LOGE(TAG, "T2 FAIL: checksum corrompido foi aceito");
            ++failed;
        } else {
            ESP_LOGI(TAG, "T2 checksum corrompido REJECT: PASS");
        }
    }

    // ---- T3: truncada (sem '*') --------------------------------------
    {
        const char* t3 = "$GPRMC,123519,A,4807.038";
        if (validate_checksum(t3, std::strlen(t3))) {
            ESP_LOGE(TAG, "T3 FAIL: truncada foi aceita");
            ++failed;
        } else {
            ESP_LOGI(TAG, "T3 truncada REJECT: PASS");
        }
    }

    // ---- T4: lat=0, lon=0 -> no_fix ----------------------------------
    // Checksum gerado em runtime para acoplar ao validador (auto-consistencia).
    {
        const char* body = "GPRMC,000000,V,0000.0000,N,00000.0000,E,000.0,000.0,010100,,";
        char t4[128];
        uint8_t cs = 0;
        for (const char* q = body; *q; ++q) cs ^= static_cast<uint8_t>(*q);
        std::snprintf(t4, sizeof(t4), "$%s*%02X", body, cs);
        const size_t t4_len = std::strlen(t4);

        if (!validate_checksum(t4, t4_len)) {
            ESP_LOGE(TAG, "T4 FAIL: checksum auto-gerado nao valida (bug em runtime)");
            ++failed;
        } else {
            if (lock_take()) { g_snap = GpsSnapshot{}; lock_give(); }
            parse_rmc(t4, t4_len);
            bool nofix = false;
            if (lock_take()) { nofix = g_snap.no_fix; lock_give(); }
            if (!nofix) {
                ESP_LOGE(TAG, "T4 FAIL: lat=0,lon=0,status=V nao gerou no_fix=true");
                ++failed;
            } else {
                ESP_LOGI(TAG, "T4 lat=0,lon=0 -> no_fix: PASS");
            }
        }
    }

    // ---- Veredicto ---------------------------------------------------
    if (failed > 0) {
        ESP_LOGE(TAG, "%d self-test(s) falharam — abortando.", failed);
        std::abort();
    }
    ESP_LOGI(TAG, "self-tests: TODOS PASS (T1..T4).");

    // Reset estado poluido pelos self-tests
    if (lock_take()) {
        g_snap = GpsSnapshot{};
        g_snap.status = '?'; g_snap.ns = '?'; g_snap.ew = '?'; g_snap.no_fix = true;
        g_stats = GpsStats{};
        g_last_rmc[0] = '\0';
        lock_give();
    }
}

}  // namespace

// =========================================================================
extern "C" void app_main(void)
{
    g_boot_us = esp_timer_get_time();
    g_mtx = xSemaphoreCreateMutex();
    if (g_mtx == nullptr) {
        ESP_LOGE(TAG, "mutex create FAIL");
        return;
    }
    // sentinela inicial
    g_snap.status = '?';
    g_snap.ns     = '?';
    g_snap.ew     = '?';
    g_snap.no_fix = true;

    ESP_LOGI(TAG, "==== GPS NEO-6M READER — Diag Fase 0.3 ====");
    ESP_LOGI(TAG, "UART_NUM_%d  RX=GPIO%d  TX=GPIO%d  baud=%d",
             GPS_UART_PORT, GPS_RX_GPIO, GPS_TX_GPIO, GPS_BAUDRATE);

    run_self_tests();

    if (esp_err_t e = uart_init(); e != ESP_OK) {
        ESP_LOGE(TAG, "uart_init FAIL: %s", esp_err_to_name(e));
        return;
    }

    // Console REPL em UART0 (coexiste com ESP_LOGx)
    esp_console_repl_t* repl = nullptr;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "drone> ";
    esp_console_dev_uart_config_t uart_cli_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cli_cfg, &repl_cfg, &repl));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_cli();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    xTaskCreatePinnedToCore(rx_task,       "gps_rx",       4096, nullptr, 12, nullptr, 0);
    xTaskCreatePinnedToCore(reporter_task, "gps_reporter", 4096, nullptr,  6, nullptr, 0);

    ESP_LOGI(TAG, "Setup OK. Aguardando sentencas NMEA…");
    ESP_LOGI(TAG, "CLI: digite 'help' p/ comandos. Cold-start indoor pode levar ~3 min.");

    // app_main termina; tasks seguem rodando.
}
