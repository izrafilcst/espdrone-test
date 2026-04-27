# claude.md — Manifesto Técnico do Projeto Drone ESP32-S3

> **Persona ativa:** Engenheiro de Sistemas Críticos · Especialista em Hardware de Potência · Arquiteto de Sistemas Embarcados
> **Stack:** ESP-IDF v5.x sobre PlatformIO · C++17 sobre FreeRTOS SMP · Toolchain `xtensa-esp32s3-elf`
> **Target MCU:** `esp32-s3-wroom-1` variante **N16R8** (16 MB Flash QIO + 8 MB PSRAM **Octal**)

Este documento é a **fonte única de verdade** para o agente Claude Code operar no repositório. Todas as decisões de pinagem, concorrência e build estão aqui consolidadas. Qualquer divergência entre código e este manifesto é considerada bug.

---

## 1. Escopo & Filosofia do Projeto

Construímos um quadricóptero/drone híbrido com:

- **Plataforma de voo (Drone):** ESP32-S3-WROOM-1 N16R8 + câmera OV2640 + MPU6050 + GPS NEO-6M + 4× motores DC 4.2V/3A acionados por MOSFETs IRLML6344 com diodos SS14 de roda livre.
- **Controle dedicado (Hardware):** ESP32 (DevKit comum) + display 320×240 SPI (ILI9341/ST7789) + 2 joysticks analógicos. Comunicação **ESP-NOW** (latência < 5 ms, frame de 32 bytes a 100 Hz).
- **Controle complementar (Mobile):** Smartphone conectado em **Wi-Fi AP** do drone para receber stream **MJPEG** (porta 80, endpoint `/stream`).
- **Energia:** 1 célula Li-ion 4.2V (nominal 3.7V) com barramento principal dimensionado para **12 A contínuos**, 18 A de pico (200 ms).

Princípios não-negociáveis:

1. **Determinismo > otimização prematura.** O loop de voo (PID) deve fechar em ≤ 1 ms, sempre, mesmo durante streaming de vídeo.
2. **Failsafe first.** Toda perda de comunicação por > 500 ms aciona descida controlada (rampa negativa de throttle).
3. **Quem manda no Core 1 é o voo.** Networking, vídeo e BLE jamais executam em Core 1.

---

## 2. Estrutura de Diretórios (ESP-IDF + PlatformIO)

```
drone-esp32s3/
├── platformio.ini              # Config PIO: env esp32-s3-devkitc-1, framework=espidf
├── sdkconfig.defaults           # Defaults globais (PSRAM Octal, WDT, FreeRTOS SMP)
├── sdkconfig.defaults.esp32s3   # Overrides específicos do target
├── partitions.csv               # Tabela custom: app 4MB, ota_0/1, spiffs, coredump
├── claude.md                    # ESTE arquivo (manifesto)
├── guia_progresso.md            # Roteiro de fases & skills
├── README.md
│
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp                 # Bootstrap: NVS, eventos, spawn das tasks
│   ├── app_config.hpp           # #defines globais e versão de firmware
│   └── pinmap.hpp               # ÚNICA fonte de pinagem (espelha §3 deste doc)
│
├── components/
│   ├── flight_engine/           # [Core 1] Loop PID, fusão sensorial, hover
│   │   ├── include/
│   │   │   ├── pid_controller.hpp
│   │   │   ├── madgwick_filter.hpp
│   │   │   ├── flight_state_machine.hpp
│   │   │   └── motor_mixer.hpp
│   │   ├── src/
│   │   │   ├── pid_controller.cpp
│   │   │   ├── madgwick_filter.cpp
│   │   │   ├── flight_state_machine.cpp
│   │   │   └── motor_mixer.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── motor_driver/            # MCPWM 4 canais + soft-start + sag compensation
│   │   ├── include/motor_driver.hpp
│   │   ├── src/motor_driver.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── imu_mpu6050/             # I2C1, calibração offline e DMP opcional
│   ├── gps_neo6m/               # UART2, parser NMEA RMC/GGA, fix detection
│   ├── power_monitor/           # ADC1 com média móvel + Kalman 1D para VBAT
│   │
│   ├── networking/              # [Core 0] ESP-NOW + Wi-Fi AP coexistência
│   │   ├── include/
│   │   │   ├── espnow_link.hpp
│   │   │   ├── wifi_ap.hpp
│   │   │   └── camera_streamer.hpp
│   │   ├── src/
│   │   │   ├── espnow_link.cpp
│   │   │   ├── wifi_ap.cpp
│   │   │   └── camera_streamer.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── camera_ov2640/           # esp32-camera wrapper, JPEG hardware
│   ├── telemetry_bus/           # Mutex-protected shared state singleton
│   └── cli_serial/              # Console UART0 + comandos via esp_console
│
├── test/
│   ├── unit/                    # Catch2 / Unity para componentes desacoplados
│   └── integration/             # HIL com pytest-embedded
│
├── tools/
│   ├── trace_width_calc.py      # IPC-2221 calculator (ver §10)
│   ├── flash_drone.sh
│   └── monitor.sh
│
└── docs/
    ├── pcb/                     # Schematics + Gerber
    ├── thermo/                  # Análise térmica de MOSFETs
    └── safety/                  # FMEA, checklist pré-voo
```

### platformio.ini (referência canônica)

```ini
[env:drone-s3]
platform = espressif32@^6.6.0
board = esp32-s3-devkitc-1
framework = espidf
monitor_speed = 115200
monitor_filters = esp32_exception_decoder, time
upload_speed = 921600
build_flags =
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue
    -DCONFIG_ESPTOOLPY_FLASHSIZE_16MB=1
    -DCONFIG_SPIRAM_MODE_OCT=1
    -std=gnu++17
board_build.partitions = partitions.csv
board_upload.flash_size = 16MB
board_build.arduino.memory_type = qio_opi   ; flash QIO + PSRAM OPI (Octal)
```

### sdkconfig.defaults (linhas obrigatórias)

```
# PSRAM Octal — CRÍTICO para N16R8
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y

# FreeRTOS SMP
CONFIG_FREERTOS_UNICORE=n
CONFIG_FREERTOS_HZ=1000

# Watchdogs (CRÍTICO p/ tarefas de longa duração)
CONFIG_ESP_TASK_WDT=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
CONFIG_ESP_INT_WDT=y
CONFIG_ESP_INT_WDT_TIMEOUT_MS=300

# Camera
CONFIG_OV2640_SUPPORT=y
CONFIG_CAMERA_TASK_STACK_SIZE=4096

# Wi-Fi co-existência
CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=16
CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=32
```

---

## 3. Mapeamento de Pinagem — Tabela Rígida (ESP32-S3 N16R8)

> ⚠️ **Atenção crítica:** GPIO **26-32** (Flash interno) e GPIO **33-37** (linhas de dados da PSRAM Octal) são **fisicamente reservados** no módulo WROOM-1 N16R8. Mesmo que o pinout do módulo os exponha, **qualquer escrita causa corrupção de PSRAM e crash imediato.**

### 3.1 Pinos PROIBIDOS (não usar nunca)

| GPIO | Função interna |
|------|----------------|
| 26-32 | SPI Flash interno (QIO) |
| 33-37 | **PSRAM Octal (8 linhas de dados)** ← N16R8 specific |
| 19, 20 | USB-OTG D-/D+ (uso somente se USB desabilitado) |
| 43, 44 | UART0 console (debug — manter livre) |

### 3.2 Pinos de Strapping (usar com cautela)

| GPIO | Strap | Mitigação |
|------|-------|-----------|
| 0 | Boot mode (LOW = download) | Pull-up externo 10kΩ; sem cargas pesadas |
| 3 | JTAG source | Pode ser usado como GPIO comum após boot |
| 45 | VDD_SPI voltage | Deixar default (pull-down interno) |
| 46 | Boot source | Pode ser usado, mas evitar em sinais críticos |

### 3.3 Pinagem Definitiva do Drone

| Função | GPIO | Periférico | Notas |
|--------|------|------------|-------|
| **Motores (MCPWM Group 0)** | | | PWM 20 kHz, 13-bit |
| M1 — Frente Esquerda (CCW) | **38** | MCPWM0_OPR0_A | Saída para gate IRLML6344 |
| M2 — Frente Direita (CW) | **39** | MCPWM0_OPR0_B | Resistor série 47Ω no gate |
| M3 — Traseira Direita (CCW) | **40** | MCPWM0_OPR1_A | Pull-down 10kΩ no gate |
| M4 — Traseira Esquerda (CW) | **41** | MCPWM0_OPR1_B | RC snubber opcional |
| **Câmera OV2640** | | | DVP paralelo 8-bit |
| XCLK (clock para câmera) | **15** | LEDC ch0 @ 20 MHz | |
| SIOD (SCCB SDA) | **4** | I2C0_SDA | Bus interno do sensor |
| SIOC (SCCB SCL) | **5** | I2C0_SCL | |
| VSYNC | **6** | DVP | |
| HREF | **7** | DVP | |
| PCLK | **13** | DVP | |
| Y2 (D0) | **11** | DVP | |
| Y3 (D1) | **9** | DVP | |
| Y4 (D2) | **8** | DVP | |
| Y5 (D3) | **10** | DVP | |
| Y6 (D4) | **12** | DVP | |
| Y7 (D5) | **18** | DVP | |
| Y8 (D6) | **17** | DVP | |
| Y9 (D7) | **16** | DVP | |
| PWDN | -1 | NC | Não conectado |
| RESET | -1 | NC | Reset por software |
| **IMU MPU6050** | | | I2C1 a 400 kHz |
| SDA | **47** | I2C1_SDA | Pull-up 4.7kΩ p/ 3.3V |
| SCL | **48** | I2C1_SCL | Pull-up 4.7kΩ p/ 3.3V |
| INT (data ready) | **2** | GPIO IRQ rising | Trigger de leitura @ 1kHz |
| **GPS NEO-6M** | | | UART2 a 9600 bps (default), upgrade p/ 38400 |
| TX (ESP→GPS, opcional) | **42** | UART2_TX | Para enviar UBX config |
| RX (GPS→ESP) | **14** | UART2_RX | NMEA RMC/GGA |
| **Energia** | | | |
| VBAT_SENSE (ADC) | **1** | ADC1_CH0 | Divisor 100k:33k → 0–4.2V mapeado |
| **Indicadores** | | | |
| LED_STATUS (RGB on-board) | **21** | RMT (WS2812) | LED do DevKit |
| **Reservados (futuro)** | | | |
| AUX_GPIO | **3** | — | Disponível, strap mas estável após boot |

### 3.4 Conferência cruzada PSRAM Octal

Os pinos GPIO **33, 34, 35, 36, 37** estão **proibidos** e **não aparecem** em nenhuma linha da tabela acima. Esta é uma asserção verificável: qualquer PR que reintroduza esses GPIOs deve ser rejeitada em CI por linter de pinagem (`tools/lint_pinmap.py`).

---

## 4. Distribuição de Cores (FreeRTOS SMP)

### 4.1 Diagrama de afinidade

```
┌─────────────────────────────────────┐    ┌─────────────────────────────────────┐
│            CORE 0 (PRO)             │    │            CORE 1 (APP)             │
│         "Networking Core"           │    │          "Flight Engine"            │
├─────────────────────────────────────┤    ├─────────────────────────────────────┤
│ • Wi-Fi driver (prio 23, sistema)   │    │ • Sensor Read Task   (prio 23)      │
│ • lwIP / TCP-IP    (prio 18)        │    │   ↳ MPU6050 @ 1 kHz via INT GPIO2   │
│ • ESP-NOW RX/TX    (prio 22)        │    │ • PID Loop Task      (prio 22)      │
│ • Camera Streamer  (prio 10)        │    │   ↳ período 1 ms (xTaskDelayUntil)  │
│ • BLE (opcional)   (prio  9)        │    │ • Motor Driver Task  (prio 21)      │
│ • CLI Serial       (prio  5)        │    │ • State Machine      (prio 20)      │
│ • Telemetry TX     (prio  8)        │    │ • Madgwick Filter (inline no PID)   │
│ • Idle 0           (prio  0)        │    │ • Idle 1             (prio  0)      │
└─────────────────────────────────────┘    └─────────────────────────────────────┘
```

### 4.2 Tabela canônica de tasks

| Task | Core | Prioridade | Stack | Período | Justificativa |
|------|------|-----------|-------|---------|---------------|
| `task_pid_loop` | 1 | 22 | 8 kB | 1 ms (1 kHz) | Determinismo absoluto; isolada de IRQ Wi-Fi |
| `task_sensor_read` | 1 | 23 | 6 kB | event-driven (INT) | Maior prio que PID p/ garantir dado fresco |
| `task_motor_drive` | 1 | 21 | 4 kB | 1 ms | Consome saída do PID via fila |
| `task_state_machine` | 1 | 20 | 4 kB | 10 ms | Failsafe, hover, transitions |
| `task_espnow_rx` | 0 | 22 | 4 kB | event-driven | Comandos do controle hardware |
| `task_telemetry_tx` | 0 | 8 | 4 kB | 100 ms | Status para HUD do controle |
| `task_camera_stream` | 0 | 10 | 8 kB | 33 ms (~30 fps) | MJPEG HTTP |
| `task_gps_parser` | 0 | 12 | 4 kB | event (UART) | NMEA → estrutura compartilhada |
| `task_power_monitor` | 0 | 7 | 3 kB | 100 ms | ADC + Kalman p/ VBAT |
| `task_cli` | 0 | 5 | 6 kB | event-driven | Debug interativo |

### 4.3 Por que essa distribuição?

- **Core 1 dedicado ao voo:** Wi-Fi/lwIP rodam exclusivamente no Core 0. Isso evita que um burst de pacotes TCP destrua a janela de 1 ms do PID.
- **PID com prio 22 (não 24):** A `task_sensor_read` (prio 23) precisa preemptar para entregar dado fresco. Se invertido, o PID rodaria com dado obsoleto.
- **Motor Driver separado:** Permite implementar **soft-start** e **slew-rate limiting** sem poluir o PID.

---

## 5. Diretrizes de Concorrência

### 5.1 Estado compartilhado: `TelemetryBus` (singleton thread-safe)

```cpp
// components/telemetry_bus/include/telemetry_bus.hpp
class TelemetryBus {
public:
    static TelemetryBus& instance();

    struct Snapshot {
        float roll, pitch, yaw;       // graus
        float altitude_m;
        float vbat_v;
        float gps_lat, gps_lon;
        uint16_t throttle[4];         // duty 0..8191 (13-bit)
        uint32_t loop_us;             // duração do último PID
        uint8_t flight_mode;
        bool gps_fix;
    };

    Snapshot get();                   // copia atomicamente sob mutex
    void update_attitude(float r, float p, float y);
    void update_motors(const uint16_t (&duty)[4]);
    // ...

private:
    Snapshot data_{};
    SemaphoreHandle_t mtx_;           // recursive mutex, timeout 1 ms
};
```

**Regras:**

1. Toda leitura/escrita do `Snapshot` passa pelo mutex.
2. Timeout do `xSemaphoreTake` é **1 ms** — falha em pegar não bloqueia o PID; loga e segue com último valor.
3. Nunca segurar mutex enquanto chama `printf`, `vTaskDelay` ou rede.
4. Ordem de aquisição (quando múltiplos): `mtx_telemetry → mtx_motor → mtx_gps`. Sempre.

### 5.2 Watchdog & vTaskDelay

- Toda task que faz loop sem bloqueio nativo **deve** chamar `vTaskDelayUntil(&last, period)` ou `taskYIELD()`.
- O **TWDT (Task Watchdog)** está habilitado em ambos os cores com timeout 5s. Tasks de longa duração registram-se com `esp_task_wdt_add()` e fazem `esp_task_wdt_reset()` a cada iteração.
- O **IWDT (Interrupt WDT)** está em 300 ms — qualquer ISR que demore mais que isso é bug.

### 5.3 Filas e eventos

- Comandos ESP-NOW → `xQueueSend` para `cmd_queue` (size 4, drop-oldest).
- IMU INT → `xTaskNotifyGiveFromISR` para `task_pid_loop`.
- **Não usar event groups** para sinais críticos de tempo real — `TaskNotify` é ~3× mais rápido.

### 5.4 Memória

- Buffers de câmera (≥ 100 KB) **vão para PSRAM** via `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`.
- Estruturas de PID, estado, mutexes ficam em **DRAM interna** — acesso < 5 ns vs ~30 ns na PSRAM.
- Stacks de tasks de Core 1 **nunca** em PSRAM (cache invalidation custa ~70 ciclos).

---

## 6. Estratégias de Superação dos Desafios

### 6.1 Brownout & Voltage Sag

Bateria Li-ion sob carga de 12A apresenta queda dinâmica devido à resistência interna (~80–150 mΩ). O PID precisa **compensar** a perda de empuxo.

**Solução:** termo de feed-forward no `motor_mixer`:

```cpp
// V_nominal = 4.0V (target). V_atual lido do power_monitor.
// Empuxo ∝ V², logo correção = (V_nom / V_atual)²
float sag_compensation = (V_NOMINAL * V_NOMINAL) / (vbat_filtered * vbat_filtered);
sag_compensation = std::clamp(sag_compensation, 1.0f, 1.35f);  // teto 35%

for (int i = 0; i < 4; ++i) {
    duty[i] = std::clamp(int(pid_out[i] * sag_compensation), 0, MAX_DUTY);
}
```

Também: configurar `CONFIG_ESP_BROWNOUT_DET_LVL_SEL_7` (2.44V) e capacitor eletrolítico **1000 µF low-ESR** no barramento principal (ver §10).

### 6.2 EMI / Ruído eletromagnético

12A pulsando a 20 kHz nos MOSFETs gera **dB/dt** brutal. O MPU6050 fica a centímetros disso.

**Mitigações em camadas:**

1. **Layout PCB:** loop de corrente do motor o mais curto possível. Plano de terra contínuo sob trilhas de potência.
2. **Filtro RC no SDA/SCL do I2C1:** 100 Ω + 100 pF para ground, próximo ao MPU6050.
3. **Cabo blindado** do MPU6050 ao MCU se montagem física separada; shield aterrado em **um único ponto** (lado do MCU).
4. **Software:** no driver MPU6050, usar **filtro DLPF interno em 41 Hz** (registro CONFIG = 0x03). Cortes mais agressivos introduzem latência de fase no PID.
5. **Ferrite bead** (BLM18EG471SN1D, 470 Ω @ 100 MHz) na linha 3.3V que alimenta o IMU.

### 6.3 Latência de rádio vs encoder de vídeo

O JPEG encoder do OV2640 pode segurar a CPU. Comandos de controle **não podem esperar.**

**Hierarquia rígida:**

- ESP-NOW RX gera ISR → `task_espnow_rx` (prio 22) preempta `task_camera_stream` (prio 10).
- Buffer de câmera é entregue ao webserver via **fila com drop-oldest** (size 2). Se o cliente Wi-Fi está lento, frames são descartados — controle nunca trava.
- O `task_camera_stream` faz `vTaskDelay(0)` após cada frame para forçar yield ao scheduler.

---

## 7. Padrões de Código (C++ OO sobre FreeRTOS)

### 7.1 Princípios

- **RAII em recursos do kernel:** wrapper `Mutex`, `Queue<T,N>`, `Task` que gerenciam handles automaticamente.
- **Sem `new`/`delete` em runtime crítico.** Pré-aloque tudo no construtor.
- **Sem exceções.** Compilar com `-fno-exceptions -fno-rtti`. Erros via `esp_err_t` ou `std::expected`-like.
- **Métodos de loop em static thunk:**

```cpp
class PIDController {
public:
    void start() {
        xTaskCreatePinnedToCore(taskThunk, "pid", 8192, this,
                                22, &handle_, /*core=*/1);
    }
private:
    static void taskThunk(void* arg) {
        static_cast<PIDController*>(arg)->run();
    }
    void run() { /* loop principal */ }
    TaskHandle_t handle_ = nullptr;
};
```

### 7.2 Soft-Start (rampa de aceleração)

A bateria Li-ion **odeia** spikes de corrente do estado parado. O motor parado tem corrente de partida ~3× nominal. Com 4 motores: pico de ~36 A num barramento de 12 A. Inaceitável.

**Solução:** rampa linear de 0 → setpoint em **800 ms** na transição `IDLE → ARMED → THROTTLE_UP`.

```cpp
// motor_driver.cpp
constexpr uint32_t SOFT_START_MS = 800;
uint16_t soft_ramp(uint16_t target, uint32_t elapsed_ms) {
    if (elapsed_ms >= SOFT_START_MS) return target;
    return uint16_t((uint64_t(target) * elapsed_ms) / SOFT_START_MS);
}
```

Adicionalmente, **slew-rate limit** entre iterações do PID (delta máximo de 50 unidades de duty / ms) para proteger química da célula em manobras agressivas.

### 7.3 CLI via Serial (esp_console)

Comandos mínimos para debug em campo (porta UART0, 115200):

| Comando | Ação |
|---------|------|
| `arm` / `disarm` | Liga/desliga motores |
| `motor <id> <duty>` | Teste individual de motor (somente em DISARMED+TEST_MODE) |
| `imu calib` | Inicia calibração de offsets |
| `pid set <axis> <kp> <ki> <kd>` | Tuning ao vivo |
| `telemetry` | Snapshot atual |
| `reboot` | Reset por software |
| `dump pinmap` | Imprime tabela §3 — verificação de sanidade |

---

## 8. Fluxo de Build & Flash

```bash
# Build
pio run -e drone-s3

# Flash + monitor (USB-CDC)
pio run -e drone-s3 -t upload -t monitor

# Apenas monitor (com decoder de exception)
pio device monitor --filter esp32_exception_decoder

# Limpar
pio run -e drone-s3 -t clean

# Análise de tamanho
pio run -e drone-s3 -t size
```

### 8.1 Coredump em campo

Particionar para coredump (`partitions.csv` inclui `coredump, data, coredump, ,64K`).
Após crash:

```bash
pio run -t coredump -e drone-s3
# gera relatório com backtrace + registradores
```

---

## 9. Checklist de Sanidade (executar antes de cada PR)

- [ ] `tools/lint_pinmap.py` passa (nenhum GPIO 26-37 referenciado).
- [ ] `pio check` (cppcheck) sem warnings em `flight_engine/`.
- [ ] Nenhuma alocação dinâmica em `task_pid_loop` (verificar com `heap_caps_get_free_size` antes/depois).
- [ ] `vTaskDelay` ou `xTaskDelayUntil` presente em todo loop de task.
- [ ] Mutex de telemetria com timeout finito (jamais `portMAX_DELAY` no Core 1).
- [ ] Compilação com `-Werror -Wall -Wextra` limpa.
- [ ] Testes unitários de `pid_controller` e `madgwick_filter` passam.

---

## 10. Anexo A — Cálculos de Referência Rápida

### Largura de trilha (IPC-2221, externa, 1oz)

`I = 0.048 · ΔT^0.44 · A^0.725` → resolvendo para A em mil²:

| Corrente | ΔT = 10°C | ΔT = 20°C |
|----------|-----------|-----------|
| 3 A (motor branch) | **1.2 mm** | 0.74 mm |
| 6 A | 3.1 mm | 1.9 mm |
| **12 A (main bus)** | **10.2 mm** | 6.3 mm |

→ Conclusão: **barramento principal exige copper pour ou camada 2oz** (ver §11 do `guia_progresso.md`).

### Potência dissipada no MOSFET (IRLML6344)

- `Rds(on) @ Vgs=3.3V ≈ 50 mΩ` (datasheet curva típica)
- `P_cond = I² · Rds(on) = 3² · 0.050 = 0.45 W` por motor
- Soma 4 motores: 1.8 W. Em SOT-23 (RθJA ≈ 250°C/W sem cobre extra), ΔT ≈ **112°C** → **inviável sem dissipação ampliada**.
- **Solução:** copper pour de 200 mm² em cada dreno → RθJA cai para ~80°C/W → ΔT ≈ 36°C → ok.

### PWM dos motores

- f_pwm = **20 kHz** (acima do audível, abaixo de switching loss excessivo)
- Resolução = `160 MHz / 20 kHz = 8000` → ~13 bits efetivos no MCPWM.

---

## 11. Versionamento deste documento

| Versão | Data | Autor | Mudança |
|--------|------|-------|---------|
| 1.0.0  | 2026-04-25 | Claude (gen) | Versão inicial — pinagem, SMP, build, mitigações |

> Qualquer alteração de pinagem, prioridade de task ou frequência de PWM **exige bump de versão** e nota de migração.