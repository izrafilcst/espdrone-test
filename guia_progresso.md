# guia_progresso.md — Roteiro Faseado · Drone ESP32-S3 N16R8

> **Stack pedagógico:** Antigravity Awesome Skills v7.x (`https://github.com/sickn33/antigravity-awesome-skills`)
> **Invocação:** Cada prompt nesta guia usa a notação `@skill-name` reconhecida por Claude Code, Cursor, Antigravity IDE, Gemini CLI e similares. As skills são markdown files que ensinam o agente a executar tarefas específicas com qualidade industrial.
> **Pré-requisito:** Leia `claude.md` antes desta guia. Toda decisão técnica aqui referencia diretamente seções do manifesto.

Este documento divide o projeto em **4 fases progressivas**. Cada fase tem:

1. **Objetivo mensurável** (definition of done).
2. **Skills Antigravity recomendadas** (com justificativa).
3. **Prompts de execução** prontos para colar no agente.
4. **Cálculos / tabelas de referência** quando aplicável.
5. **Critérios de saída** para avançar à fase seguinte.

---

## Fase 0 — Firmwares de Diagnóstico (MVP Unitário)

> **Filosofia:** Antes de fazer qualquer coisa voar, cada periférico precisa funcionar **isoladamente** e ser caracterizado em bancada. Esta fase produz 4 firmwares descartáveis que viram fixtures de regressão.

### 0.1 Objetivos

- Provar I2C respondendo (MPU6050 retorna `WHO_AM_I = 0x68`).
- Provar capacidade de drive de **3 A em burst** nos MOSFETs com queda de tensão < 200 mV.
- Receber e parsear sentenças NMEA do GPS (RMC + GGA) com cálculo de fix.
- Capturar 1 frame JPEG do OV2640 a 320×240 e enviá-lo via UART (ou salvar em SPIFFS para inspeção).

### 0.2 Skills recomendadas

| Skill | Por quê |
|-------|---------|
| `@brainstorming` | Estruturar a bateria de testes antes de escrever código |
| `@test-driven-development` | Cada diagnóstico vira um teste verde/vermelho determinístico |
| `@debugging-strategies` | Bisect sistemático quando algo falha (fundamental aqui) |
| `@logging-best-practices` | Saída serial padronizada para captura automatizada |
| `@embedded-systems-c` | Padrões idiomáticos C/C++ em ESP-IDF |
| `@hardware-debugging` | Protocolo de uso de osciloscópio/multímetro nos pontos certos |

### 0.3 Prompt 0.1 — Scanner I2C

```
Use @brainstorming + @embedded-systems-c para gerar um firmware standalone
em ESP-IDF C++17 chamado "diag_i2c_scanner.cpp" que:

1. Inicializa I2C_NUM_1 em GPIO47 (SDA) e GPIO48 (SCL) a 400 kHz.
2. Faz varredura completa 0x03..0x77 com timeout de 100ms por endereço.
3. Imprime via UART0 (115200) tabela formatada estilo `i2cdetect`.
4. Específicamente reporta:
   - PASS se 0x68 respondeu (MPU6050 padrão).
   - PASS alternativo se 0x69 respondeu (AD0 puxado).
   - FAIL com sugestões de troubleshooting (pull-ups, alimentação, fios).
5. Imediatamente após detecção, lê WHO_AM_I (registro 0x75) e valida == 0x68.
6. Repete a varredura a cada 2 segundos para detectar loose contacts.

Use @logging-best-practices para padronizar tags ESP_LOGI/ESP_LOGE.
Não use componentes — código todo em main.cpp para facilitar bring-up.
```

**Critério de sucesso:** log estável de `WHO_AM_I = 0x68 PASS` por 30 s sem falsos negativos.

### 0.4 Prompt 0.2 — Burst de 3A nos MOSFETs

```
Use @embedded-systems-c + @hardware-debugging para gerar
"diag_mosfet_burst.cpp" que caracteriza UM motor por vez via MOSFET IRLML6344.

Especificações:
- MCPWM unit 0, timer 0, operator 0, generator A → GPIO38 (motor M1).
- f_pwm = 20 kHz, resolução 13 bits (period count = 8000).
- Sequência de teste:
   1. Soft-start linear de 0 → 50% duty em 800 ms.
   2. Mantém 50% por 3 segundos (burst nominal ~1.5A, dependendo do motor).
   3. Salto súbito a 100% por 200 ms (BURST DE 3A — caracteriza Vds e Tj).
   4. Rampa descendente em 1 segundo até 0.
   5. Aguarda 5 segundos. Repete.
- Durante o burst, lê ADC1_CH0 (GPIO1, divisor para VBAT) a cada 1 ms
  e calcula sag (V_idle - V_burst).
- Imprime relatório a cada ciclo:
   * V_idle, V_burst_min, sag_mV, t_recovery_ms.

Importante:
- Use @logging-best-practices: prefixo "[MOSFET-M1]" em todos logs.
- Adicione gate driver protection: resistor série 47Ω no design — comentar no código.
- Habilitar TaskWDT com timeout 6s (rampa + burst + recovery cabe folgado).

Antes do código, gere uma checklist de bancada com @hardware-debugging:
o que medir com osciloscópio (Vgs, Vds, Vbat ripple), onde colocar pontas.
```

**Critério de sucesso:** sag < 200 mV em burst de 200 ms; Vgs limpo (sem ringing > 4V) no scope; MOSFET frio ao toque após 50 ciclos.

### 0.5 Prompt 0.3 — Leitura NMEA do GPS

```
Use @embedded-systems-c + @test-driven-development para gerar
"diag_gps_nmea.cpp" que valida o NEO-6M.

Requisitos:
- UART2 em GPIO14 (RX) e GPIO42 (TX), 9600-8N1 default.
- Buffer circular de 1024 bytes para sentenças NMEA.
- Parser inline (sem libs externas) para:
   * $GPRMC → extrair UTC time, status (A/V), lat, lon, speed, course, date.
   * $GPGGA → extrair fix quality (0..2), num sats, altitude.
- Validação de checksum XOR de cada sentença antes de aceitar.
- A cada 1 segundo imprime resumo:
   [GPS] sats=X fix=Y lat=DDMM.mmmm lon=DDDMM.mmmm alt=ZZ.Zm

Casos de teste (rodar no construtor antes do loop):
- Sentença RMC válida hardcoded → todos campos extraídos corretos.
- Sentença com checksum corrompido → REJECT.
- Sentença truncada → REJECT.
- Sentença com lat 0,0 (sem fix) → flag no_fix=true.

Use @debugging-strategies para gerar comandos de teste em CLI:
"gps stats" → uptime, sentences_total, sentences_valid, sentences_rejected.
"gps last_rmc" → última sentença bruta recebida.
```

**Critério de sucesso:** dentro de casa (cold start) em ≤ 3 minutos com vista parcial do céu; outdoor ≤ 60 s; precisão lat/lon estável (variação < 5 m em 5 min de medida).

### 0.6 Prompt 0.4 — Captura de Frame da Câmera

```
Use @embedded-systems-c para gerar "diag_camera_frame.cpp" que valida
end-to-end o pipeline OV2640 → DVP → JPEG hardware → PSRAM.

Configuração obrigatória:
- esp_camera_config_t com pinagem EXATA da §3.3 do claude.md.
- pixel_format = PIXFORMAT_JPEG.
- frame_size = FRAMESIZE_QVGA (320×240) — manter pequeno na fase 0.
- jpeg_quality = 12 (10–15 é range típico).
- fb_count = 2 (double-buffer em PSRAM).
- fb_location = CAMERA_FB_IN_PSRAM (CRÍTICO — evita estouro de DRAM).
- grab_mode = CAMERA_GRAB_LATEST.

Loop principal:
1. esp_camera_fb_get() — captura frame.
2. Valida: fb != nullptr, fb->len > 1024 (sanity check), bytes 0..1 == 0xFF 0xD8 (SOI JPEG).
3. Imprime: [CAM] frame=N len=BYTES width=W height=H elapsed_ms=T.
4. A cada 10 frames, dump primeiros 64 bytes em hex (verificar header JPEG correto).
5. esp_camera_fb_return().
6. Mede FPS médio em janelas de 30 frames.

Critério adicional — usar @hardware-debugging:
- Se fb == nullptr 3× consecutivas → printar diagnóstico:
  "verificar: alimentação 3.3V no XCLK_PIN, SCCB pull-ups, aterramento,
   antenas paralelas (Y2-Y9), continuidade VSYNC/HREF/PCLK".

Não usar Wi-Fi nesta fase — é puro teste do front-end de imagem.
```

**Critério de sucesso:** ≥ 25 FPS sustentados em QVGA por 60 segundos; primeiros bytes sempre `FF D8`; nenhum frame com `len < 2 KB`.

### 0.7 Critérios de saída da Fase 0

- [ ] 4 firmwares passam isoladamente em bancada com bateria carregada.
- [ ] Logs de cada teste arquivados em `docs/diagnostics/phase0/`.
- [ ] Foto de osciloscópio do burst de 3A salva (Vgs, Vds, Vbat).
- [ ] Coordenadas GPS validadas contra Google Maps com erro < 10 m.
- [ ] Frame JPEG da câmera renderizado corretamente em viewer externo.

---

## Fase 1 — Hardware & Termodinâmica (PCB Real, Prova de Sobrevivência)

> **Filosofia:** Firmware perfeito sobre hardware ruim falha em voo. Esta fase entrega um PCB que **não derrete sob 12A**, com guia de montagem que qualquer técnico (ou nós mesmos depois de uma noite mal dormida) consegue seguir.

### 1.1 Objetivos

- PCB de 4 camadas (ou 2 com pour generoso) capaz de 12A contínuos.
- Cálculo de trilha rigoroso (IPC-2221), com margem documentada.
- Esquemático com **star grounding** correto e capacitância de bypass dimensionada.
- Guia de montagem human-readable, passo-a-passo, com fotos de referência (placeholders OK).
- Análise térmica dos MOSFETs (RθJA, ΔT esperado, mitigação).

### 1.2 Skills recomendadas

| Skill | Por quê |
|-------|---------|
| `@architecture` | Visão sistêmica do PCB como subsistema |
| `@doc-coauthoring` | Guia de montagem em prosa clara, não jargonizada |
| `@brainstorming` | Antecipar modos de falha (FMEA leve) |
| `@safety-compliance` | Checklist de segurança elétrica |
| `@code-review` (adaptado) | Revisão peer-style do esquemático |

### 1.3 Cálculo Canônico — Largura de Trilha (IPC-2221, Externa, 1oz)

Fórmula: $I = k \cdot \Delta T^{0.44} \cdot A^{0.725}$, com $k = 0.048$ para trilhas externas.

Resolvendo para área $A$ em mil² com 1oz copper (espessura 1.378 mil):

#### Tabela 1.3.A — Resultado completo

| Corrente | ΔT = 10°C | ΔT = 20°C | ΔT = 30°C | Recomendação |
|----------|-----------|-----------|-----------|--------------|
| 1 A | 0.45 mm (18 mil) | 0.28 mm (11 mil) | 0.21 mm (8 mil) | sinais e alimentação leve |
| 3 A (motor branch) | **1.18 mm (47 mil)** | 0.74 mm (29 mil) | 0.55 mm (22 mil) | **adotar 1.5 mm** com margem |
| 6 A (intermediário) | 3.07 mm (121 mil) | 1.93 mm (76 mil) | 1.43 mm (56 mil) | adotar 3.5 mm |
| **12 A (main bus)** | **10.16 mm (400 mil)** | 6.39 mm (252 mil) | 4.74 mm (187 mil) | **inviável como trilha** → usar copper pour 2oz |

**Decisão de projeto:** o barramento de 12A será implementado como **plano de cobre dedicado** numa camada interna (em PCB 4 camadas) ou como **pour de 2oz** com largura efetiva ≥ 8 mm em PCB 2 camadas. Trilhas de 3A (cada motor) ficam em **1.5 mm** = 30% de margem sobre o mínimo IPC-2221 a 10°C.

#### Tabela 1.3.B — Verificação por queda de tensão (V drop)

Para uma trilha de 50 mm de comprimento, ρ_Cu = 1.7e-8 Ω·m, área = 1.5mm × 0.035mm = 5.25e-8 m²:

$R = \rho \cdot L / A = 1.7\text{e-8} \cdot 0.05 / 5.25\text{e-8} = 0.016 \, \Omega$

Queda em 3A: $V_{drop} = 3 \cdot 0.016 = 48 \, mV$ → **aceitável** (< 1.2% de 4.0V).

### 1.4 Prompt 1.1 — Esquemático Crítico

```
Use @architecture + @safety-compliance para gerar uma descrição
detalhada do esquemático do drone (texto + listas, sem ferramenta CAD)
que cubra os 5 blocos:

[BLOCO A] Power Distribution
  - Conector XT30 ou JST-PH para bateria Li-ion 4.2V.
  - Fusível polifuse PPTC 15A (resetável).
  - Capacitância no barramento principal:
      * 1× 1000 µF / 6.3V eletrolítico low-ESR (Panasonic FR ou Nichicon HW).
      * 2× 100 µF cerâmico X5R 1210.
      * 4× 10 µF cerâmico X7R 0805 distribuídos perto dos drains dos MOSFETs.
  - LDO AMS1117-3.3 ou TPS62133 (buck) para 3.3V do MCU/sensores.
      * 22 µF entrada, 10 µF saída.

[BLOCO B] MCU + Periféricos digitais
  - ESP32-S3-WROOM-1 N16R8 conforme pinagem §3 claude.md.
  - Botões BOOT (GPIO0) e RESET com pull-ups e debounce capacitor 100nF.
  - Conector USB-C (5V → não passar por LDO; D+/D- direto).

[BLOCO C] Driver de Motores (×4 idênticos)
  - MOSFET IRLML6344 (SOT-23) low-side.
  - Resistor gate-source 10kΩ (pull-down, evita gate flutuante).
  - Resistor gate série 47Ω (limita ringing).
  - Diodo SS14 schottky de roda livre, ânodo no dreno, cátodo no +V_motor.
  - Capacitor de bypass 100nF cerâmico bem próximo de cada conjunto motor+MOSFET.

[BLOCO D] Sensores
  - MPU6050 com pull-ups I2C 4.7kΩ para 3.3V no MPU side.
  - Ferrite bead BLM18EG471SN1D na linha 3.3V do MPU (470Ω @ 100MHz).
  - Filtro RC 100Ω + 100pF nas linhas SDA/SCL próximas ao MPU.
  - GPS NEO-6M com seu próprio LDO (cap reservatório 10µF + 100nF).
  - Antena cerâmica passiva ou conector U.FL para externa.

[BLOCO E] Interface
  - LED RGB on-board (já no WROOM-1 GPIO48? Não — usar GPIO21 com WS2812 externo se necessário).
  - Header de programação 6 pinos (GND, 3.3V, EN, IO0, RX0, TX0).
  - Ponto de teste TP-VBAT, TP-3V3, TP-GND para multímetro.

Para cada bloco, use @safety-compliance para listar:
- O que pode dar errado (failure mode).
- Como detectar em bench test.
- Como mitigar.

Saída: documento markdown "schematic_brief.md" com BOM consolidada (parts + Mouser/LCSC PNs).
```

### 1.5 Prompt 1.2 — Guia Human-Readable de Montagem (Star Grounding + Caps)

```
Use @doc-coauthoring para escrever "assembly_guide.md", em português
brasileiro, em tom didático mas técnico, voltado a quem está montando
o protótipo pela primeira vez.

Cobrir nesta ordem:

1. PREPARAÇÃO
   - Lista de ferramentas (ferro 60W ponta fina, multímetro RMS, lupa,
     álcool isopropílico, malha dessoldadora).
   - ESD: pulseira aterrada, base antiestática.
   - Limpeza prévia da PCB com IPA + escova macia.

2. ORDEM DE MONTAGEM (do menor pro maior, do mais delicado pro mais robusto):
   a. Resistores 0805 e 1206.
   b. Capacitores cerâmicos 0603-1210.
   c. MOSFETs IRLML6344 e diodos SS14 (orientação crítica — explicar
      marca do diodo em detalhe).
   d. CIs em SOT-23 e SOIC (LDO).
   e. WROOM-1 (módulo grande — pré-aquecer a placa em chapa quente
      ou retrabalhar com hot air a 280°C).
   f. Capacitor eletrolítico 1000µF (CRÍTICO: polaridade — explicar
      faixa branca = negativo).
   g. Conectores e headers.

3. STAR GROUNDING — explicar com analogia:
   "Imagine que a corrente é água escoando para um único ralo (o GND
    da bateria). Se múltiplas linhas de motor disputam o mesmo caminho
    de retorno por uma trilha estreita, elas geram quedas de tensão
    que outros componentes (MPU6050) interpretam como ruído.
    A solução é: cada bloco de alta corrente tem seu próprio caminho
    de retorno até um único ponto físico, chamado star ground."

   Mostrar com diagrama ASCII:

      [Motor1]──┐
      [Motor2]──┤
      [Motor3]──┼──→ [STAR GND] ←── [GND da bateria]
      [Motor4]──┤        ↑
                         │
      [MCU GND]  ────────┘
      [SENSOR GND] ──────┘
      [LDO GND] ─────────┘

   Regra prática: linhas de retorno dos motores NUNCA compartilham
   trilha com linhas de retorno do sensor.

4. POSICIONAMENTO DE CAPACITORES
   - 1000µF principal: entre + da bateria e star ground, MAIS PRÓXIMO
     POSSÍVEL do conector da bateria. Distância máxima 15mm em trilha.
   - 100µF cerâmicos: dois, em paralelo com o 1000µF, mas físicamente
     espalhados (um em cada extremidade do barramento) para reduzir
     loop de corrente AC.
   - 100nF dos motores: a no máximo 5mm do dreno do MOSFET correspondente.
     Trilha grossa e curta.
   - 10µF do barramento: distribuídos a cada ~25mm ao longo do bus.
   - Para cada par de blocos diferentes (motor vs sensor), DESACOPLAR
     com caps de freq. diferentes (1µF + 100nF + 10nF) cobrindo várias
     décadas de frequência.

5. INSPEÇÃO PRÉ-ENERGIZAÇÃO
   Antes de ligar a bateria PELA PRIMEIRA VEZ:
   - Multímetro em modo continuidade: testar VBAT vs GND (deve ser
     resistência alta — se beep, há curto, NÃO LIGUE).
   - Verificar polaridade do conector da bateria.
   - Bench power supply com limite de corrente em 500 mA, sobe tensão
     gradual de 0 → 4.0V e observa consumo. Se passar de 200 mA com
     MCU ainda apagado, ABORTAR.

6. PRIMEIRA INICIALIZAÇÃO
   - USB conectado (alimenta MCU sem precisar da bateria).
   - Flashar firmware diag_i2c_scanner da fase 0.
   - Validar console serial.
   - SÓ ENTÃO conectar bateria e medir VBAT_SENSE no ADC.

7. TROUBLESHOOTING (top 5 problemas comuns)
   a. MCU não enumera USB: GPIO19/20 com cargas? CONFIG_USB habilitado?
   b. PSRAM crash: CONFIG_SPIRAM_MODE_OCT está =y? Algum GPIO 33-37 sendo escrito?
   c. I2C não acha MPU: pull-ups instalados? VCC do MPU é 3.3V (não 5V)?
   d. Motor não gira: gate driver com 47Ω? IRLML6344 orientação correta?
      (Source pino 1, Gate pino 2, Drain pino 3).
   e. Reset durante motor burst: brownout. Aumentar 1000µF? Bateria fraca?

Adicionar GLOSSÁRIO no fim com termos: ESR, slew rate, freewheeling diode,
gate charge, dV/dt immunity. Cada termo em 1 frase clara.
```

### 1.6 Prompt 1.3 — Análise Térmica do MOSFET

```
Use @architecture (modo análise) para gerar "thermal_analysis.md".

Cenário:
- IRLML6344 em SOT-23, RθJA típico = 250°C/W (datasheet, sem dissipação extra).
- I_motor = 3 A contínuo.
- Vgs = 3.3V → Rds(on) ≈ 50 mΩ (pior caso até 80 mΩ a 100°C).
- Hard switching @ 20 kHz, Qg ≈ 7.4 nC.

Calcular:
1. Perda de condução: P_cond = I² · Rds(on) → escalar com temperatura.
2. Perda de switching: P_sw = 0.5 · Vds · Id · (tr + tf) · f_pwm.
   * tr ≈ tf ≈ 30 ns (datasheet).
3. Perda total por MOSFET, depois somar 4× para o barramento.
4. Temperatura de junção esperada: Tj = Tamb + P · RθJA.
5. Determinar área mínima de copper pour no dreno para baixar RθJA
   para nível seguro (Tj < 80°C com Tamb 25°C).

Referenciar tabela de RθJA vs área de cobre:
   sem pour:     250 °C/W
   50  mm²:      ~140 °C/W
   100 mm²:      ~100 °C/W
   200 mm²:      ~80 °C/W   ← target
   400 mm²:      ~60 °C/W

Concluir com recomendação de design:
- "Cada MOSFET deve ter copper pour de 200 mm² no dreno, conectado por
   pelo menos 4 vias térmicas a um plano de cobre na camada inferior."

Adicionar derating: voar em ambiente 35°C ou enclosure abafado piora
tudo em ~10°C. Documentar.
```

### 1.7 Critérios de saída da Fase 1

- [ ] Esquemático revisado por 2 olhos (peer review estilo `@code-review`).
- [ ] BOM com PNs reais (Mouser/Digikey/LCSC) e custo total documentado.
- [ ] PCB roteada respeitando larguras da §1.3.
- [ ] Análise térmica indicando Tj < 80°C em pior caso.
- [ ] Guia de montagem testado por uma pessoa diferente do projetista.
- [ ] Bench test: bateria 4.0V, 4 motores em soft-start a 50% por 30s — VBAT cai no máx 0.4V (sag dinâmico < 10%).

---

## Fase 2 — Firmware de Voo (Intelligence)

> **Filosofia:** O hardware está sólido, agora dotamos o drone de inteligência. PID estável, IMU calibrado, hover digno deste nome.

### 2.1 Objetivos

- PID em 3 eixos (roll, pitch, yaw) rodando a 1 kHz com jitter < 50 µs.
- Calibração automática de offsets do IMU em ≤ 8 segundos no boot.
- Filtro de Madgwick fundindo accel + gyro com β ajustável.
- Modo Hover: drone mantém altitude (estimada por integração de aceleração com correção de pressão se BMP280 for adicionado, ou puramente atitude+throttle fixo na v1) e atitude neutra sem comando do operador.
- Soft-start, slew limit e sag compensation funcionais.

### 2.2 Skills recomendadas

| Skill | Por quê |
|-------|---------|
| `@control-systems` | Sintonia clássica de PID (Ziegler-Nichols, mão pesada justificada) |
| `@real-time-systems` | Garantia de jitter, prioridades, análise WCET |
| `@test-driven-development` | PID e Madgwick são puro estado→estado, ideal para testes unitários |
| `@performance-optimization` | Manter loop em 1 ms quando todo o resto roda |
| `@debugging-strategies` | Bisect quando o drone "não fica parado" |
| `@code-review` | Voar com bug é caro |

### 2.3 Prompt 2.1 — Calibração Automática do IMU

```
Use @control-systems + @test-driven-development para implementar
"imu_calibrator.cpp" como componente independente.

Algoritmo:
1. Boot detecta drone em superfície plana (média móvel de gyro < 0.5°/s
   por 1 segundo). Se em movimento, espera.
2. Coleta 4096 amostras de accel (x,y,z) e gyro (x,y,z) a 500 Hz.
3. Calcula:
   - bias_gyro = média das 4096 leituras (gyro deve ser zero parado).
   - bias_accel = média - (0,0,g) onde g = 1.0 em unidades de 1g.
   - Calcula desvio padrão para detectar movimento (rejeita calibração
     se σ_gyro > 1°/s ou σ_accel > 0.05g).
4. Salva offsets em NVS (namespace "imu", keys "bx_g", "by_g", ...).
5. Boot subsequente: lê NVS direto, recalibra apenas se temperatura
   variou > 10°C desde última calibração (TEMP_OUT do MPU6050).

Testes unitários (rodar no host com Catch2):
- Stub que retorna leituras com bias conhecido → calibrador deve
  zerá-lo a < 0.01°/s.
- Stub que retorna ruído σ alto → calibrador rejeita.
- Carregar e salvar NVS → round-trip exato.

Comandos CLI:
- "imu calib" → força recalibração.
- "imu offsets" → imprime valores atuais.
- "imu reset" → zera NVS, força recalibração no próximo boot.
```

### 2.4 Prompt 2.2 — Filtro de Madgwick

```
Use @control-systems + @real-time-systems + @performance-optimization
para implementar "madgwick_filter.cpp" otimizado para Xtensa LX7.

Forma matemática (referência: Sebastian Madgwick, 2010):
  q̇_w = -0.5 · (q × ω) - β · ∇f

Onde:
  q = quaternion de orientação (4 floats)
  ω = velocidade angular do gyro (rad/s)
  β = ganho de correção (típico 0.033 a 0.1)
  ∇f = gradiente da função de erro do accel

Otimizações OBRIGATÓRIAS:
- Usar `float` (não double) — FPU do S3 é single precision em hardware.
- Pré-computar 1/sqrt usando aproximação Quake (rsqrt) — atinge ~3% erro
  mas executa em 12 ciclos vs 60+ da libm.
- Tudo em DRAM interna (atributo IRAM_ATTR no método `update()`).
- Sem alocação dinâmica — quaternion é membro da classe.

Interface:
class MadgwickFilter {
public:
    void update(float gx, float gy, float gz,    // rad/s
                float ax, float ay, float az,    // gs (qualquer unidade,
                                                 //     será normalizado)
                float dt);                       // segundos
    void getEuler(float& roll, float& pitch, float& yaw) const;
    void setBeta(float beta);                    // tuning ao vivo via CLI
private:
    float q0_=1, q1_=0, q2_=0, q3_=0;
    float beta_ = 0.041f;  // default Madgwick original
};

Testes unitários:
- Entrada de gyro estática → quaternion não muda.
- Accel = (0,0,1g), gyro = 0 → após 1s, getEuler retorna (0,0, qualquer).
- Rotação puramente em yaw a 90°/s, dt=1ms → após 1s, yaw ≈ 90°.
- Estresse: rodar 1 milhão de updates, medir tempo total → profiling.

Métrica alvo: update() < 8 µs no S3 a 240 MHz.
```

### 2.5 Prompt 2.3 — PID Controller

```
Use @control-systems + @test-driven-development para implementar
"pid_controller.cpp" — PID parametrizado por eixo.

API:
class PID {
public:
    struct Params { float kp, ki, kd, i_max, out_min, out_max; };
    PID(const Params& p);
    float update(float setpoint, float measurement, float dt);
    void reset();
    void setParams(const Params& p);  // tuning ao vivo
private:
    Params p_;
    float integral_ = 0;
    float prev_error_ = 0;
    float prev_d_ = 0;     // p/ filtro derivativo
};

Detalhes não-óbvios mas críticos:
1. ANTI-WINDUP: clamp do integral em ±i_max ANTES de somar à saída.
   Forma "back-calculation" pode ser implementada na v2.
2. DERIVATIVO COM FILTRO: derivative kick em mudança brusca de setpoint
   é fatal para drone. Implementar como "derivative on measurement"
   (não no erro), com filtro IIR de 1 polo em ~30 Hz:
       d_filtered = α · d_raw + (1-α) · d_filtered_prev
       α = dt / (RC + dt), RC = 1/(2π·30) ≈ 0.0053s
3. SATURAÇÃO E CONDITIONAL INTEGRATION: se a saída está saturada
   E o erro tem mesmo sinal do incremento integral, NÃO incrementar.
4. Reset do integral em transição DISARMED → ARMED.

Sintonia inicial (primeiro voo, drone amarrado em fios — captive flight):
   roll, pitch:  Kp=0.8, Ki=0.5, Kd=0.05
   yaw:          Kp=2.0, Ki=0.3, Kd=0.0  (sem D em yaw)

Testes unitários no host:
- Resposta a degrau com modelo de planta de 1ª ordem (τ=100ms).
   Deve atingir setpoint em < 500ms sem overshoot > 10%.
- Anti-windup: setpoint impossível → integral saturado em i_max.
- Derivative kick: degrau em setpoint → saída derivativa filtrada NÃO spika.
```

### 2.6 Prompt 2.4 — Motor Mixer + Soft Start + Sag Compensation

```
Use @real-time-systems + @code-review para implementar "motor_mixer.cpp".

Mixer quadricóptero X (frente entre M1 e M2):
   M1 = throttle - pitch + roll - yaw   (frente esquerda, CCW)
   M2 = throttle - pitch - roll + yaw   (frente direita, CW)
   M3 = throttle + pitch - roll - yaw   (traseira direita, CCW)
   M4 = throttle + pitch + roll + yaw   (traseira esquerda, CW)

Após mistura, aplicar EM ORDEM:
1. Sag compensation (claude.md §6.1):
   gain = clamp((V_NOM/V_atual)², 1.0, 1.35)
   M_i *= gain
2. Slew rate limit:
   ΔM_max = 50 unidades / iteração de 1ms
   M_i = clamp(M_i, M_prev_i - ΔM_max, M_prev_i + ΔM_max)
3. Soft-start (em transição armed):
   if (now - arm_time) < 800ms:
     M_i *= ((now - arm_time) / 800ms)
4. Saturação final:
   M_i = clamp(M_i, IDLE_DUTY, MAX_DUTY)
   IDLE_DUTY = 100 (motor não desliga em hover, evita re-spool delay).

Aplicar via MCPWM:
   mcpwm_comparator_set_compare_value(comp[i], M_i);

Logs (em modo TEST, não em voo real):
   [MIX] T=850 R=12 P=-3 Y=1 → M=[820, 854, 866, 832] sag=1.08
```

### 2.7 Prompt 2.5 — Modo Hover

```
Use @control-systems + @debugging-strategies para implementar
"flight_state_machine.cpp" com modo HOVER.

Estados: DISARMED → ARMED_IDLE → THROTTLE_UP → IN_FLIGHT → HOVER → LANDING → DISARMED.

HOVER é entrado quando:
  - drone está em IN_FLIGHT, e
  - operador soltou ambos joysticks por > 500ms (deadband ±5%).

Em HOVER, o controlador faz:
  - Setpoint roll = 0, pitch = 0 (atitude neutra).
  - Setpoint yaw = yaw atual no momento de entrar (não rotaciona).
  - Throttle: HOLD do valor médio dos últimos 500ms antes de entrar
    em hover. Sem altímetro, isso é o melhor que conseguimos em v1
    (drone afunda lentamente — operador interrompe HOVER, ajusta).

Failsafe em HOVER:
  - Se VBAT < 3.4V: força LANDING (rampa decrescente de throttle 80% → IDLE em 4s).
  - Se sem comando ESP-NOW por > 1500ms: força LANDING.
  - Se gyro report > 500°/s em qualquer eixo: KILL motors (proteção tumbling).

CLI:
  "fly state" → estado atual.
  "fly hover_force" → entra em hover mesmo com joystick deslocado.
  "fly land" → força LANDING.
  "fly kill" → mata motores imediatamente (apenas em DISARMED ou emergência).

Use @debugging-strategies para gerar evento de log em CADA transição:
  [FSM] DISARMED → ARMED_IDLE  reason=arm_command  vbat=4.05V
  [FSM] ARMED_IDLE → THROTTLE_UP  reason=throttle>10%
  [FSM] IN_FLIGHT → HOVER  reason=joystick_idle_500ms
```

### 2.8 Critérios de saída da Fase 2

- [ ] Captive flight (drone preso em fios elásticos) com PID estável: roll/pitch oscilam < ±5° em hover.
- [ ] Loop PID com `loop_us` reportado em telemetria — média < 700 µs, p99 < 950 µs.
- [ ] Calibração de IMU automática funciona em < 8s no boot.
- [ ] Madgwick converge em < 2s com drone em movimento brusco e volta à posição correta em < 1s após parar.
- [ ] Hover sem controle por 10s mantém atitude (drone preso ainda) com erro < 8°.
- [ ] Failsafe testados: cortar Wi-Fi de comando → LANDING engaja em < 2s.

---

## Fase 3 — Integração Híbrida (ESP-NOW + Wi-Fi AP Coexistentes)

> **Filosofia:** Vídeo é importante, controle é vital. O sistema híbrido garante que perda de Wi-Fi nunca degrade o controle.

### 3.1 Objetivos

- ESP-NOW funcionando em paralelo com Wi-Fi AP no mesmo rádio.
- Stream MJPEG via HTTP a ≥ 25 FPS QVGA, ≥ 15 FPS VGA.
- Latência de comando ESP-NOW ≤ 8 ms (50% percentil), ≤ 20 ms (p99).
- Failsafe robusto: timeout de comando, queue overflow handling, packet loss > 30% trigger warning.

### 3.2 Skills recomendadas

| Skill | Por quê |
|-------|---------|
| `@api-design-principles` | Protocolo binário ESP-NOW bem versionado |
| `@network-protocols` | Coexistência ESP-NOW + Wi-Fi, canal compartilhado |
| `@security-auditor` | AP exposto é superfície de ataque |
| `@frontend-design` | UX do HUD no controle dedicado |
| `@performance-optimization` | Latência percentil, não média |
| `@debugging-strategies` | Sniffing de pacotes, análise de jitter |

### 3.3 Coexistência ESP-NOW + Wi-Fi AP — Restrição CRÍTICA

ESP-NOW e Wi-Fi compartilham o mesmo rádio. **Ambos DEVEM operar no MESMO CANAL** ou um suspende o outro com latência catastrófica.

**Solução adotada:**

- Wi-Fi AP do drone configurado em **canal 6** (fixo).
- ESP-NOW também em canal 6 — `esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE)` no boot do drone, **e do controle dedicado também**.
- Modo Wi-Fi do drone: `WIFI_MODE_AP` (não APSTA — economiza recursos).

### 3.4 Prompt 3.1 — Protocolo ESP-NOW

```
Use @api-design-principles para definir o protocolo binário ESP-NOW
em "espnow_protocol.hpp".

Frame de COMANDO (controle → drone), 32 bytes total:
  uint8_t  magic;         // 0xA5 (sanity)
  uint8_t  version;       // 0x01
  uint8_t  seq;           // contador 0..255 (para detectar perdas)
  uint8_t  flags;         // bit0=arm, bit1=hover_request, bit2=land,
                          //   bit3=killswitch, demais reservados
  int16_t  roll;          // -1000..+1000 (mapeado p/ ±30°)
  int16_t  pitch;
  int16_t  yaw;
  uint16_t throttle;      // 0..1000
  uint8_t  reserved[18];  // futura expansão
  uint16_t crc16;         // CCITT-FALSE sobre os 30 bytes anteriores

Frame de TELEMETRIA (drone → controle), 32 bytes:
  uint8_t  magic;         // 0xA6
  uint8_t  version;
  uint8_t  seq_echo;      // último seq recebido do controle
  uint8_t  flags;         // bit0=armed, bit1=hover_active, bit2=fix_gps,
                          //   bit3=low_battery, bit4=landing
  int16_t  roll, pitch, yaw;
  uint16_t altitude_cm;
  uint16_t vbat_mv;       // milivolts
  uint16_t loop_us;
  int32_t  lat_e7;        // latitude × 1e7
  int32_t  lon_e7;
  uint16_t crc16;

Implementar:
- crc16_ccitt() em IRAM, branchless (table lookup 256 entries).
- Validação completa em RX: magic, version, crc, seq monotônico
  (warn se gap > 3, accept; reject se gap > 50 — provável replay).

Endereçamento:
- Controle aprende MAC do drone via "broadcast pairing" no primeiro boot
  (drone broadcast a cada 100ms até receber confirmação, 30s timeout).
- Pareamento salvo em NVS, ambos lados.
- Reset de pareamento: pressionar BOOT por 5s no controle.

Métricas a expor via CLI:
  "espnow stats" → tx_total, tx_ok, tx_fail, rx_total, rx_valid, rx_crc_err
                   ms_since_last_rx, packet_loss_pct (janela 1s)
```

### 3.5 Prompt 3.2 — WebServer MJPEG

```
Use @network-protocols + @performance-optimization para implementar
"camera_streamer.cpp".

Endpoints HTTP (esp_http_server, multi-thread default):
  GET /        → página HTML simples com <img src="/stream"> + status
  GET /stream  → multipart/x-mixed-replace, MJPEG
  GET /status  → JSON com vbat, fps, uptime
  POST /cmd    → recebe JSON (futura) — em v1 retorna 501 Not Implemented

Configuração HTTP:
  - max_open_sockets = 4
  - core_id = 0 (afinidade Core 0)
  - stack_size = 8192
  - lru_purge_enable = true (drop client antigo se 5º se conectar)

Loop /stream:
  while (cliente conectado):
      fb = esp_camera_fb_get()
      if (!fb) continue
      // Header MIME multipart
      httpd_resp_send_chunk(req, "--frame\r\n"
                            "Content-Type: image/jpeg\r\n"
                            "Content-Length: %d\r\n\r\n",
                            fb->len)
      httpd_resp_send_chunk(req, fb->buf, fb->len)
      httpd_resp_send_chunk(req, "\r\n", 2)
      esp_camera_fb_return(fb)
      // CRÍTICO: yield p/ permitir ESP-NOW preemptar
      vTaskDelay(pdMS_TO_TICKS(1))

Otimizações:
- jpeg_quality = 12 default; cair para 18 se VBAT < 3.6V (qualidade
  pior = arquivos menores = menos rádio = menos consumo).
- Se >2 clientes conectados, reduzir framesize automaticamente.
- Se task_pid_loop reporta loop_us > 950 (cabeçalho amarelo), pausar
  stream por 100ms — controle SEMPRE prevalece.

Configuração Wi-Fi AP:
  ssid:      "DRONE_<últimos 4 hex do MAC>"
  password:  "drone1234" em primeira inicialização (forçar troca via CLI no campo)
  channel:   6 (fixo, ver §3.3)
  max_connection: 2
  beacon_interval: 100ms (default)

Use @security-auditor para listar mitigações:
- WPA2 obrigatório (nunca open).
- DHCP server interno, range 192.168.4.10 - 192.168.4.20.
- Logs cada associação/desassociação com MAC para auditoria.
- Endpoint /cmd só aceita request com header X-Drone-Token configurado em runtime.
```

### 3.6 Prompt 3.3 — Priorização de IRQ Rádio sobre Encoder de Vídeo

```
Use @real-time-systems + @debugging-strategies para auditar e ajustar
prioridades.

Comportamento desejado:
- Pacote ESP-NOW recebido → ISR Wi-Fi (não controlamos prio do driver,
  é alta) → enfileira em cmd_queue → task_espnow_rx (prio 22) acorda
  → preempta task_camera_stream (prio 10).
- Frame de câmera nunca bloqueia o controle.

Auditoria a executar:
1. Habilitar `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y` e
   `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`.
2. Comando CLI "rtos top" imprime tabela:
     Task                    Core  Prio   CPU%   Stack-watermark
     task_pid_loop             1    22    18%    3120
     task_camera_stream        0    10    35%    5800
     task_espnow_rx            0    22     2%    3400
     ...
3. Rodar cenário de stress:
   - Stream MJPEG com 1 cliente celular.
   - Enviar comandos ESP-NOW a 100 Hz por 60s.
   - Medir tempo entre RX e processamento (timestamp na ISR vs quando
     task_espnow_rx pega da queue).
4. Targets:
     p50 ISR→processamento < 200 µs
     p99 ISR→processamento < 2 ms

Se p99 estourar, mitigações em ordem:
  a. Reduzir prio de task_camera_stream para 8.
  b. Forçar yield mais frequente no streamer (a cada 1 chunk em vez
     de a cada frame).
  c. Migrar task_camera_stream para outra core? NÃO — é Core 0 por
     definição. Se Core 0 está saturado, então é hora de cortar
     features (resolução de vídeo, etc).
```

### 3.7 Prompt 3.4 — HUD do Controle Dedicado (Display 320×240)

```
Use @frontend-design + @doc-coauthoring para projetar o HUD no
display 320×240 do controle dedicado.

Layout (proposta inicial, ajustar com feedback):

  ┌──────────────────────────────────────┐
  │ DRONE_A1B3       ARMED   [HOVER]     │  ← topo: status
  ├──────────────────────────────────────┤
  │  Roll  ━━━●━━━     -3°               │
  │  Pitch ━━━●━━━     +1°               │
  │  Yaw   ━━━━●━━     +5°               │  ← atitude: barras
  │                                      │
  │      ╭─────╮     VBAT  3.92V  ▓▓▓░  │
  │      │ ALT │     RX    98%         │
  │      │ 1.2 │     LAT   -23.5505    │
  │      │  m  │     LON   -46.6333    │
  │      ╰─────╯     SAT   8           │  ← grid: GPS+power
  │                                      │
  │ T:0.65  Loop:740µs   Pkt:312        │  ← rodapé: telemetria
  └──────────────────────────────────────┘

Implementação:
- LVGL (Light Versatile Graphics Library) v8.x — a melhor stack
  embarcada GUI para ESP32.
- Render em parcial dirty-rect (não redesenhar tela inteira a cada frame).
- 30 FPS suficiente — display SPI a 40MHz consegue.
- Cores: fundo #000814 (azul escuro), texto #00d4ff (ciano),
  warnings #ff4757 (vermelho), success #2ed573 (verde).

Estados visuais:
- ARMED em verde, DISARMED em cinza, FAILSAFE em vermelho piscante.
- VBAT < 3.6V: ícone de bateria vermelho piscante.
- Pacotes perdidos > 5% janela 1s: ícone Wi-Fi em amarelo.
- GPS sem fix: GPS_FIX em vermelho.

Use @doc-coauthoring para escrever também "operator_manual.md" cobrindo:
- Sequência de boot (controle primeiro, drone depois — pareamento).
- Significado de cada cor/estado.
- Procedimento de pareamento de novo drone.
- O que fazer se RX% cair: distância, obstáculo, interferência?
- Procedimento de pouso de emergência manual (botão LAND).
```

### 3.8 Critérios de saída da Fase 3

- [ ] Voo livre (sem amarras) em ambiente protegido — drone responde a comandos do controle dedicado por 60s sem glitch.
- [ ] Cliente celular conectado simultaneamente recebe stream a ≥ 25 FPS QVGA.
- [ ] Latência ESP-NOW: p50 < 8ms, p99 < 20ms (medido com instrumentação no firmware).
- [ ] Teste de Wi-Fi loss: derrubar AP do drone enquanto streaming ativo → controle ESP-NOW segue 100% funcional.
- [ ] Teste de range: voo a 30m em campo aberto sem perda > 2% de pacotes ESP-NOW.
- [ ] Failsafe trigger: simular perda de comando 1.5s → drone entra em LANDING automaticamente.
- [ ] Operator manual revisado por usuário não-técnico que conseguiu armar+desarmar sem assistência.

---

## Síntese Final: Mapa Mental do Projeto

```
                    ┌────────────────────────────────┐
                    │   FASE 0 — DIAGNÓSTICO          │
                    │   "cada peça funciona?"         │
                    │   skills: brainstorming, TDD,   │
                    │           debug, hardware       │
                    └─────────────┬──────────────────┘
                                  │ all-green
                                  ▼
                    ┌────────────────────────────────┐
                    │   FASE 1 — HARDWARE/THERMO      │
                    │   "vai sobreviver a 12A?"       │
                    │   skills: architecture, doc,    │
                    │           safety                │
                    └─────────────┬──────────────────┘
                                  │ PCB validada
                                  ▼
                    ┌────────────────────────────────┐
                    │   FASE 2 — VOO INTELLIGENTE     │
                    │   "PID + Madgwick + Hover"      │
                    │   skills: control-systems,      │
                    │           real-time, TDD        │
                    └─────────────┬──────────────────┘
                                  │ captive flight ok
                                  ▼
                    ┌────────────────────────────────┐
                    │   FASE 3 — INTEGRAÇÃO HÍBRIDA   │
                    │   "ESP-NOW + Wi-Fi coexistem"   │
                    │   skills: api-design, network,  │
                    │           security, frontend    │
                    └────────────────────────────────┘
                                  │
                                  ▼
                              FREE FLIGHT
                              MVP COMPLETO
```

---

## Anexo — Resumo das Skills Antigravity Aplicadas

| Skill | Fase de uso primária | Tipo de entrega |
|-------|----------------------|-----------------|
| `@brainstorming` | 0, 1 | Estrutura de plano antes de codar |
| `@architecture` | 1, 2 | Visão sistêmica e ADRs |
| `@test-driven-development` | 0, 2 | Tests → red → green → refactor |
| `@debugging-strategies` | 0, 2, 3 | Bisect e reprodução determinística |
| `@embedded-systems-c` | 0 | Idioms ESP-IDF e RTOS |
| `@hardware-debugging` | 0, 1 | Bench protocol com osciloscópio |
| `@logging-best-practices` | 0, 2 | Tags consistentes, níveis, persistência |
| `@doc-coauthoring` | 1, 3 | Manuais human-readable |
| `@safety-compliance` | 1 | Checklists elétricos e FMEA |
| `@control-systems` | 2 | PID, Madgwick, sintonia |
| `@real-time-systems` | 2, 3 | Determinismo, prioridades, jitter |
| `@performance-optimization` | 2, 3 | Hot path análise e redução |
| `@code-review` | 1, 2 | Peer review estruturado |
| `@api-design-principles` | 3 | Protocolo binário versionado |
| `@network-protocols` | 3 | ESP-NOW, Wi-Fi, coexistência |
| `@security-auditor` | 3 | Hardening de Wi-Fi AP exposto |
| `@frontend-design` | 3 | HUD do controle dedicado |

Para invocar uma skill em qualquer fase: comece o prompt com `Use @skill-name ...` ou `Use @skill-A + @skill-B for ...` quando precisar de combinação.

---

## Versionamento

| Versão | Data | Mudança |
|--------|------|---------|
| 1.0.0  | 2026-04-25 | Roteiro inicial: 4 fases, prompts canônicos, skills mapeadas |