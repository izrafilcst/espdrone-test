# Guia de Testes — Fase 0 (Firmwares de Bring-up)

> **Status: FASE 0 CONCLUÍDA** (2026-04-30) — todos os 4 bring-ups passaram.
> Firmware ativo no repositório: `diag_camera_frame.cpp` (Fase 0.4).
> Próximo passo: Fase 1 — PCB, térmica e montagem.

Cada firmware é um `.cpp` isolado em `main/`. Apenas um compila por vez.
Para trocar de firmware: editar **duas linhas** em `main/CMakeLists.txt` e gravar.

---

## Como Trocar de Firmware

```cmake
# main/CMakeLists.txt — ajustar SRCS e REQUIRES conforme a fase ativa

# ── Fase 0.1 ──
SRCS "diag_i2c_scanner.cpp"
REQUIRES freertos esp_system esp_driver_i2c log

# ── Fase 0.2 ──
SRCS "diag_mosfet_burst.cpp"
REQUIRES freertos esp_system esp_driver_mcpwm esp_adc log

# ── Fase 0.3 ──
SRCS "diag_gps_nmea.cpp"
REQUIRES freertos esp_system esp_driver_uart esp_timer console log

# ── Fase 0.4 ──
SRCS "diag_camera_frame.cpp"
REQUIRES freertos esp_system esp_timer log espressif__esp32-camera
```

## Comandos de Build/Flash/Monitor

```bash
# Build + flash + monitor (tudo de uma vez)
pio run -e drone-s3 -t upload -t monitor

# Só monitor (firmware já gravado)
pio device monitor --filter esp32_exception_decoder

# Apenas build (verificar compilação)
pio run -e drone-s3

# Limpar artefatos (necessário ao trocar de firmware pela primeira vez)
pio run -e drone-s3 -t clean
```

> **Dica:** sempre rodar `clean` antes do primeiro build após trocar de firmware,
> para evitar cache de objetos da fase anterior.

---

## Fase 0.1 — I2C Scanner (`diag_i2c_scanner.cpp`)

### Conexões físicas

| ESP32-S3 | MPU6050 | Notas |
|---|---|---|
| GPIO 47 (SDA) | SDA | Pull-up **4.7 kΩ → 3.3V** externo obrigatório |
| GPIO 48 (SCL) | SCL | Pull-up **4.7 kΩ → 3.3V** externo obrigatório |
| 3V3 | VCC | 3.3V **apenas** (nunca 5V no S3) |
| GND | GND | GND comum |
| GND ou 3V3 | AD0 | GND → endereço 0x68; VCC → 0x69 |

### O que observar no monitor

```
I (xxx) i2c_scan: --- Ciclo #1 ---
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
60:                               68 --  ...
I (xxx) i2c_scan: PASS: ACK em endereco PRIMARIO 0x68 (AD0=GND)
I (xxx) i2c_scan: WHO_AM_I = 0x68 — chip valido. Bring-up OK.
```

### Critério de PASS

- ACK em 0x68 (ou 0x69) **estável** em todos os ciclos de 2 s.
- `WHO_AM_I = 0x68` confirmado.

---

## Fase 0.2 — MOSFET Burst (`diag_mosfet_burst.cpp`)

### Conexões físicas

| ESP32-S3 | Circuito | Notas |
|---|---|---|
| GPIO 38 | Gate IRLML6344 via **47 Ω série** | Resiste em série no gate, não no source |
| GND | Gate via **10 kΩ pull-down** | Mantém MOSFET OFF durante boot |
| Dreno MOSFET | Motor DC 4.2V | Anodo do SS14 no dreno |
| VBAT+ | Catodo SS14 + bulk cap 1000 µF | Cap a < 5 mm do dreno |
| GPIO 1 (ADC) | Divisor 100 kΩ : 33 kΩ | Ponto médio ao GPIO; 33k ao GND; 100k ao VBAT+ |

> ⚠️ **Não conectar motor à bateria sem o capacitor de 1000 µF**.
> Sem o cap, o sag na bateria pode brownout o ESP.

### O que observar no monitor

```
I (xxx) MOSFET-M1: Ciclo #1 | V_idle=4180 mV | V_burst_min=3950 mV | sag=230 mV | t_recovery=42 ms OK
```

### Critério de PASS

| Métrica | Limite aceitável |
|---|---|
| `sag_mV` | < 200 mV → bulk cap adequado |
| `t_recovery_ms` | < 1500 ms (não TIMEOUT) |
| `v_idle_mv` | > 3500 mV (bateria carregada) |

---

## Fase 0.3 — GPS NMEA (`diag_gps_nmea.cpp`)

### Conexões físicas

| ESP32-S3 | NEO-6M | Notas |
|---|---|---|
| GPIO 14 (UART2 RX) | TX do módulo GPS | Cruzado: TX do GPS → RX do ESP |
| GPIO 42 (UART2 TX) | RX do módulo GPS | Opcional para enviar config UBX |
| 3V3 | VCC | 3.3V — módulos NEO-6M aceitam 3.3V |
| GND | GND | GND comum |

> Indoor o cold-start pode levar **2–3 minutos**. Testar próximo a janela
> ou em área aberta para fix rápido.

### Self-tests na inicialização

Antes de habilitar a UART, o firmware roda 4 testes internos.
Se qualquer um falhar: `abort()` e o ESP reinicia em loop.

```
I (xxx) GPS: self-tests: TODOS PASS (T1..T4).
```

### O que observar no monitor

```
I (xxx) GPS: [GPS] sats=7 fix=1 lat=2338.123S lon=04621.456W alt=800.5m  [FIX_OK]
```

### CLI disponível (digitar no monitor após `drone> `)

```
drone> gps stats        → uptime, total/valid/rejected
drone> gps last_rmc     → última sentença $GPRMC válida
drone> help             → lista todos os comandos
```

### Critério de PASS

- Self-tests T1..T4 passam sem `abort()`.
- `[FIX_OK]` aparece (com antena externa e céu visível).
- Contador `rejected` estável (< 5% do total).

---

## Fase 0.4 — Camera OV2640 (`diag_camera_frame.cpp`)

### Passo extra — 1ª build apenas

A Fase 0.4 depende do componente `espressif/esp32-camera` via IDF Component Manager.
Antes do primeiro build:

1. Criar o arquivo `main/idf_component.yml`:

```yaml
dependencies:
  espressif/esp32-camera:
    version: ">=2.0.4"
  idf:
    version: ">=5.0.0"
```

2. Garantir **conexão à internet** — o PlatformIO/IDF baixa o componente automaticamente.
3. Após o download (feito uma única vez), builds subsequentes são offline.

> Para voltar às Fases 0.1–0.3, **apagar** `main/idf_component.yml`
> evita que o Component Manager seja invocado desnecessariamente.

### Conexões físicas

| ESP32-S3 | OV2640 | Notas |
|---|---|---|
| GPIO 15 (XCLK) | XCLK | LEDC_TIMER_0 @ 20 MHz — medir com osciloscópio |
| GPIO 4 (SIOD) | SDA/SIOD | Pull-up **10 kΩ → 3.3V** (pull-up dedicado, não o do IMU) |
| GPIO 5 (SIOC) | SCL/SIOC | Pull-up **10 kΩ → 3.3V** |
| GPIO 6 | VSYNC | — |
| GPIO 7 | HREF | — |
| GPIO 13 | PCLK | — |
| GPIO 11 | Y2 (D0) | Barramento DVP 8-bit — usar cabos curtos (< 10 cm) |
| GPIO 9 | Y3 (D1) | |
| GPIO 8 | Y4 (D2) | |
| GPIO 10 | Y5 (D3) | |
| GPIO 12 | Y6 (D4) | |
| GPIO 18 | Y7 (D5) | |
| GPIO 17 | Y8 (D6) | |
| GPIO 16 | Y9 (D7) | |
| 3V3 | VCC | |
| GND | GND | GND comum — verificar com multímetro |
| NC | PWDN | Não conectar |
| NC | RESET | Não conectar |

### O que observar no monitor

```
I (xxx) CAM: Sensor detectado: PID=0x2642  SCCB=0x30
I (xxx) CAM: [SUSTAIN] janela iniciada — aguardando 60 s a >= 25 FPS
I (xxx) CAM: [CAM] frame=1 len=6124 width=320 height=240 elapsed_ms=3  OK
...
I (xxx) CAM: FPS (ultimos 30 frames): 28.4
...
I (xxx) CAM: [SUSTAIN] *** PASS *** 1704 frames em 60.0 s = 28.4 FPS (alvo: 25 FPS, ...)
```

### Critério de PASS

| Métrica | Limite |
|---|---|
| FPS sustentado | ≥ 25 FPS por **60 s contínuos** sem frame inválido |
| SOI de todo frame | `FF D8` (JPEG válido) |
| Tamanho mínimo | ≥ 2048 bytes por frame |

### Diagnóstico automático

Se `fb == nullptr` ocorrer 3× seguidas, o firmware imprime checklist completo com
cada pino suspeito e o que medir. Ler a seção `DIAGNOSTICO DE HARDWARE` no monitor.

---

## Resumo Rápido de Troca

| Fase | SRCS | Dependência extra |
|---|---|---|
| 0.1 | `diag_i2c_scanner.cpp` | — |
| 0.2 | `diag_mosfet_burst.cpp` | — |
| 0.3 | `diag_gps_nmea.cpp` | — |
| 0.4 | `diag_camera_frame.cpp` | `main/idf_component.yml` + internet na 1ª build |

Sequência de troca:
```bash
# 1. Editar main/CMakeLists.txt (SRCS + REQUIRES)
# 2. [Fase 0.4 apenas] criar main/idf_component.yml
pio run -e drone-s3 -t clean
pio run -e drone-s3 -t upload -t monitor
```
