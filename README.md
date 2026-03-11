# NOxDebug – NOx Sensor Monitor & Debug System

## 1. Overview

**NOxDebug** is an embedded application for **STM32F103RCT6** that acts as a **NOx (nitrogen oxides) sensor monitor and debug node**. It receives sensor data over **J1939/CAN**, supports **single- or dual-sensor** operation with configurable strategy (single / primary-backup / fusion), performs per-sensor two-segment linear calibration, drives **two independent blowback valves**, and exposes readings and parameters via **Modbus RTU**. It also acts as a Modbus master to write **4–20 mA** output values to an external current-output module.

**Main capabilities:**

- Receive NOx and O₂ data from **one or two** J1939-capable sensors: **SA 0x52** (channel 0, outlet) and **SA 0x51** (channel 1, inlet). Architecture is modular for future extension to three sensors.
- **Work mode** (register 40012 / P34): **Low byte** = mode: **0** = single (one channel), **1** = primary-backup (first valid channel), **2** = fusion (average of valid channels). **High byte** = single-channel index when mode=0: **0** = channel 0 (SA 0x52), **256 (0x0100)** = channel 1 (SA 0x51). Default mode is **primary-backup**. A single **4–20 mA** output is driven by the current strategy result (NOx + O₂).
- **Modbus register layout:** **Common** registers: NOx/O₂ output (P01/P02), output status (P07), 4–20 mA (P22/P23), work mode (P34), alarm thresholds (P12/P13). **Sensor registers:** two identical blocks (sensor 1: 40013–40050, sensor 2: 40051–40088), each with live NOx/O₂/status, calibration params, cal control, and blowback. No global “cal target”; each sensor’s cal trigger applies to that sensor.
- Per-sensor two-segment linear calibration (3 points) for NOx and O₂; each channel has its own parameters and cal trigger/point select in its sensor block. Store calibration in internal Flash (S1 + S2 = 24 floats).
- Modbus RTU **slave** on RS485 (USART1) for HMI/SCADA: read/write registers and coils.
- Modbus RTU **master** on another RS485 (UART5) to drive an external 4–20 mA output module.
- **Two blowback valves:** each sensor has the same blowback registers (interval, duration, status, countdown, command) in its block; Relay0/1 = ch0, Relay2/3 = ch1.
- OLED display for current NOx, O₂, status, and run time.

---

## 2. Hardware

| Item | Description |
|------|-------------|
| **MCU** | STM32F103RCT6 (LQFP64), 72 MHz, HSE 8 MHz + PLL×9 |
| **CAN** | 250 kbps, J1939; connects to NOx sensor |
| **USART1** | 9600 baud (default), RS485, **Modbus slave** (HMI/PC) |
| **UART5** | 9600 baud, RS485, **Modbus master** (4–20 mA module) |
| **I2C1** | OLED (PB6 SCL, PB7 SDA) |
| **SPI1** | SD card (FATFS); e.g. `config.txt` for baudrate |
| **GPIO** | Relay0/1: blowback valve **ch0** (sensor 0); Relay2/3: blowback valve **ch1** (sensor 1) |

Pin names and labels are defined in the CubeMX project (`NOx_RCT6.ioc`) and in `main.h`.

---

## 3. Software Architecture

- **HAL** + **FreeRTOS** (CMSIS-RTOS v2).
- **Tasks:**
  - **defaultTask**: placeholder.
- **NOx_Default**: calibration polling (per sensor via each sensor’s cal trigger), alarm, blowback logic (both valves), run-time display, OLED refresh.
- **ModBus_Slave**: Modbus slave poll (USART1), register and Flash access.
- **NOx_Receive**: dequeue J1939 frames (with channel index 0/1), update per-channel data (**nox_channel**), apply work-mode strategy (single / primary-backup / fusion; single channel selectable via P34 high byte), update P01/P02/P07 and per-sensor live registers (S1/S2), fill 4–20 mA buffer, send heater command on CAN.
  - **ModBus_Host**: periodically write 4–20 mA data to external slave (UART5) and read back for check.

- **Timers:** TIM6 (HAL tick), TIM7 (10 ms base for 100 ms / 1 s counters), TIM2/TIM3 (Modbus slave/host RX timeout).

Key application code lives under **USER/**; sensor math and defaults are in **nox_sensor** and **app_config.h**. Logic is split into:

- **NOx.c**: J1939 handle (per channel), tasks (NOxReceive, NOxDefault, ModBusSlave), register/Flash init, strategy (single / primary-backup / fusion).
- **nox_channel.h/c**: Per-sensor channel state and params (raw, ppm, pct, state, valid, calibration); **NoxChannel_UpdateFromCan**, **NoxChannel_GetCurrentOutput**; modular for 2 or 3 sensors.
- **blowback.c/h**: Two valves; each sensor has the same register set (interval, duration, status, countdown, command) in its block; **BLOW_CONTROL(ch, state)**; Relay0/1 and Relay2/3.
- **calibration.c/h**: NOx/O₂ calibration per sensor (each sensor’s cal trigger and point select in its block), slope/intercept, **Calibration_Init**.
- **alarm.c/h**: Alarm thresholds (P12/P13) and comparison with current output (P01/P02).

---

## 4. Communication

### 4.1 J1939 / CAN

- **Role:** Receive NOx sensor frames from **two sensors**; send one heater command for both.
- **Filter:** Frames with **source address (SA) 0x52** (channel 0, outlet) or **0x51** (channel 1, inlet) are accepted. Each frame is tagged with a channel index and pushed to the receive queue.
- **Received payload (8 bytes):**  
  - Data[0–1]: NOx raw; Data[2–3]: O₂ raw; Data[4]: status; Data[5]: heater; Data[6–7]: fault codes.
- **Heater command:** One CAN frame with ID `0x18FEDF55`, Byte7 Start-Code (e.g. 0x55) controls heating for both sensors (ATO1/ATI1 and ATO2/ATI2 per Continental interface doc).

### 4.2 Modbus Slave (USART1, RS485)

- **Default:** address `1`, baud rate **115200** (can be overridden by SD card `config.txt`).
- **Supported:** 03H (read holding), 06H (write single), 10H (write multiple), 01H/05H (coils).
- **Data format:** Holding registers are big-endian; floats use 2 consecutive 16-bit registers.

### 4.3 Modbus Master (UART5, RS485)

- Writes the **4–20 mA** values (NOx and O₂ from strategy) to an external slave (default address `0x01`, registers `REG_P01`/`REG_P02`, 2 registers), then reads them back with 03H for verification.

---

## 5. Modbus Register Map

Register layout: **common** (output, 4–20 mA, mode, alarm only), then **sensor 1 block** (40013–40050), then **sensor 2 block** (40051–40088). The two sensor blocks have **identical structure** (same number and order of registers).

### 5.1 Common Registers (40001–40012)

| Addr | Name | R/W | Description |
|------|------|-----|-------------|
| 40001–40002 | P01 | R | **Current** NOx (ppm), float — strategy output |
| 40003–40004 | P02 | R | **Current** O₂, float — strategy output |
| 40005 | P07 | R | **Current** output status word (9 bits) |
| 40006–40007 | P12 | R/W | NOx high alarm (ppm), float |
| 40008–40009 | P13 | R/W | O₂ low alarm, float |
| 40010 | P22 | R/W | 4–20 mA NOx code (written by host) |
| 40011 | P23 | R/W | 4–20 mA O₂ code (written by host) |
| 40012 | P34 | R/W | **Work mode (u16):** low byte 0=single, 1=primary-backup, 2=fusion; when single, high byte=channel (0=ch0, 256=ch1). Default 1. |

### 5.2 Sensor Block (same for sensor 1 and sensor 2)

**Sensor 1:** 40013–40050 (38 register addresses)  
**Sensor 2:** 40051–40088 (38 register addresses)

Within each block, order is:

| Offset in block | Addr (S1) | Addr (S2) | R/W | Description |
|-----------------|------------|------------|-----|-------------|
| 0–1 | 40013–40014 | 40051–40052 | R | Live NOx (ppm), float |
| 2–3 | 40015–40016 | 40053–40054 | R | Live O₂, float |
| 4 | 40017 | 40055 | R | Status word (9 bits) |
| 5–8 | 40018–40025 | 40056–40063 | R/W | Segment 1: NOx a/b, O₂ a/b (float) |
| 9–12 | 40026–40033 | 40064–40071 | R/W | Segment 2: NOx a/b, O₂ a/b (float) |
| 13–16 | 40034–40041 | 40072–40079 | R/W | Cal point 2 & 3: NOx, O₂ (float) |
| 17 | 40042 | 40080 | R/W | NOx cal trigger (0x0001 step, 0x0002 restore) |
| 18 | 40043 | 40081 | R/W | NOx point select (0/1/2) |
| 19 | 40044 | 40082 | R/W | O₂ cal trigger |
| 20 | 40045 | 40083 | R/W | O₂ point select |
| 21 | 40046 | 40084 | R/W | Blowback interval (s); 0 or 0xFFFF = stop |
| 22 | 40047 | 40085 | R/W | Blowback duration (s) |
| 23 | 40048 | 40086 | R | Blowback status (0 idle, 1 blowing) |
| 24 | 40049 | 40087 | R | Blowback countdown (s) |
| 25 | 40050 | 40088 | R/W | Blowback command (0 no op; 1 trigger once; 2 trigger+reset; 3/0xFFFF stop) |

Calibration applies **per sensor**: writing to sensor 1’s NOx/O₂ cal trigger affects sensor 1; writing to sensor 2’s affects sensor 2. There is no global “P53” calibration target.

### 5.3 Coils (01H read, 05H write)

| Coil | Description |
|------|-------------|
| D01 | Sensor 1 normal operation (Relay0). ON = normal gas path. |
| D02 | Sensor 1 blowback control (Relay1). ON = blowback valve open. |
| D03 | Sensor 2 normal operation (Relay2). ON = normal gas path. |
| D04 | Sensor 2 blowback control (Relay3). ON = blowback valve open. |
| D05 | Reserved. |

Only one sensor may be in blowback at a time (enforced in firmware).

---

## 6. How to Use

### 6.1 Build and Flash

- Open **MDK-ARM/NOx_RCT6.uvprojx** in Keil µVision.
- Build the project (F7).
- Connect the ST-Link and flash (F8). Optionally use the STM32CubeProgrammer or other tools.

### 6.2 First Power-Up

- After flash, the device loads parameters from **internal Flash** (if previously saved). If Flash is empty, defaults from **app_config.h** and **Register_Init** are used.
- **Factory / first boot:** If **ENABLE_FACTORY_FLASH_ON_EMPTY** is 1 (in **app_config.h**), on first power-up when the calibration Flash area is still erased (0xFF), the firmware **writes default calibration (24 floats) to Flash once** after **Register_Init**, so later boots load valid floats instead of NaN/FF. To re-run factory program, erase the user Flash page or call **FactoryFlash_ProgramDefaults()** after **Register_Init** from debug.
- Modbus slave uses **USART1** (RS485); ensure the HMI or PC tool uses the same baud rate (e.g. 115200) and slave address (default 1).
- Optional: put a **config.txt** on the SD card to override baud rate (e.g. `baudrate=115200`). The exact key names depend on **handleConfig()** in the SD card module.

### 6.3 Reading NOx and O₂

- **P01** / **P02** / **P07**: **Current** output (one set) — driven by work mode (single ch0, primary-backup, or fusion). Read via Modbus 03H.
- **Sensor blocks:** each sensor has **live NOx, O₂, status** at the start of its block (e.g. 40013–40017 for sensor 1, 40051–40055 for sensor 2). Use these to monitor each sensor separately.
- **P07** value **0x1FF** means all conditions OK for the current output source.

### 6.4 Calibration (3-Point, Two-Segment, Per Sensor)

- There is **no P53**. Calibration target is determined by **which sensor’s** register you write: sensor 1 block (40013–40050) or sensor 2 block (40051–40088).
- **NOx (sensor 1):**  
  - Set **40043** (NOx point select) to 0, 1, or 2.  
  - For point 1/2, set **40034–40037** / **40038–40041** (point 2/3 NOx, O₂ as float) to the reference NOx value.  
  - Apply reference gas and wait for stability.  
  - Write **40042 = 0x0001** to run the calibration step. Success: register is set to **0x000F**; failure: **0x0005**.  
  - To restore defaults: write **40042 = 0x0002**; success **0x0010**.

- **NOx (sensor 2):** same logic using **40081** (point select), **40072–40079** (point Y values), **40080** (cal trigger).

- **O₂:** use each sensor’s **O₂ point select** (40045 / 40083), **point 2/3 O₂** in the same block, and **O₂ cal trigger** (40044 / 40082) with **0x0001** (calibrate) or **0x0002** (restore).

- Calibration results are written to **internal Flash** (S1 + S2, 24 floats) so they persist after power cycle.

### 6.5 Blowback Control (Two Valves)

- **Sensor 1 (Relay0/1):** registers **40046–40050** — interval (40046), duration (40047), status R (40048), countdown R (40049), command (40050).  
  - Command: **0** = no op; **1** = trigger once; **2** = trigger + reset cycle; **3** or **0xFFFF** = stop periodic.  
  - Interval **0** or **0xFFFF** = stop periodic; **1–65534** = interval in seconds. Duration minimum 1 s.

- **Sensor 2 (Relay2/3):** registers **40084–40088** — same layout (interval, duration, status, countdown, command).

Example: set 40046 = 3600, 40047 = 60 for sensor 1 “blow 60 s every 3600 s”. Set 40084/40085 similarly for sensor 2. Write 40050 = 1 or 40088 = 1 for a single manual blow. Read status and countdown (40048/40049 and 40086/40087) for HMI display.

- **Stagger:** Periodic blowback for **sensor 2 (ch1)** is offset by **5 minutes (300 s)** from sensor 1’s phase so the two paths do not request blowback at the same tick. Ch0 still fires at `tick % interval == 0`; ch1 fires when `tick % interval == (300 % interval)` (see **BLOW_STAGGER_SEC** in **app_config.h**). Only one valve can blow at a time; if one is already blowing, the other start is skipped until it finishes.

### 6.6 Work Mode (Dual-Sensor Strategy)

- **Register 40012 (P34)** = work mode (R/W), 16-bit:
  - **Low byte (mode):**
    - **0** = **Single:** one channel drives P01/P02/P07 and 4–20 mA. Which channel is selected by the **high byte**: **0** = channel 0 (SA 0x52), **256 (0x0100)** = channel 1 (SA 0x51). Example: write **0** for single ch0, write **256** for single ch1.
    - **1** = **Primary-backup (default):** if **one path is in blowback**, output **always** uses the **other** path (P01/P02/4–20 mA). Otherwise use the first **valid** channel (0 then 1). If one channel is heating or fault (invalid), the other valid channel is used automatically. Both sensors’ data remain visible in their sensor blocks.
    - **2** = **Fusion:** average NOx and O₂ over **valid channels that are not in blowback**; if only one path is not blowing, that path is used (even if not 0x1FF). When both are usable, averaging behaves as before.

- **Default at power-up / after Register_Init:** P34 = **1** (primary-backup). Manual write to 40012 takes effect on the next strategy cycle (~50 ms).

- The **single 4–20 mA** output always reflects the current strategy result (P01/P02 = NOx and O₂). Per-sensor values are in each sensor block’s live registers for display or logging.

### 6.7 Alarms

- **P12** = NOx high alarm (ppm); **P13** = O₂ low alarm.  
- The firmware compares **current** NOx/O₂ (P01/P02) to these; alarm handling (e.g. relay or message) can be extended in **Alarm_Update()** in alarm.c.

### 6.8 4–20 mA Output

- The device converts the **current** NOx and O₂ (from the selected work mode) to 4–20 mA codes and fills **electricity_data_buf**.
- The **Modbus host** task (UART5) writes these to the external current-output slave. No extra user action is required beyond wiring and setting the slave address in **modbus_host.c** (e.g. `REG_P01`, `REG_P02`). There is **one** 4–20 mA output (NOx + O₂); per-sensor values are in the sensor blocks for display or logging.

### 6.9 Changing Defaults

- **Work mode default** is set in **USER/NOx.c** in **Register_Init()**: `g_tVar.work_mode = 1` (primary-backup). Change to `0` for single ch0 or `256` for single ch1 if needed.
- Edit **USER/app_config.h** for:
  - **NOX_SENSOR_COUNT** (currently 2), **NOX_SENSOR_COUNT_MAX** (3 for future), **NOX_SENSOR_SA_LIST**.
  - Default NOx/O₂ conversion (slope/intercept), calibration Y values, alarm thresholds.
  - Blowback default duration and interval, minimum duration, **BLOW_STAGGER_SEC** (ch1 phase offset).
  - 4–20 mA full-scale ranges and J1939 heater CAN ID/payload.

---

## 7. File Layout (Summary)

| Path | Purpose |
|------|--------|
| **USER/app_config.h** | Central defaults, **NOX_SENSOR_COUNT**, work mode enum, J1939 constants. |
| **USER/nox_sensor.h, nox_sensor.c** | Calibration math, raw→ppm/%, 4–20 mA conversion. |
| **USER/nox_channel.h, nox_channel.c** | Per-sensor channel state and params; **NoxChannel_UpdateFromCan**, **NoxChannel_GetCurrentOutput**; modular for 2/3 sensors. |
| **USER/NOx.h, NOx.c** | J1939 handling (per channel), tasks, strategy, register/Flash init; uses blowback, calibration, alarm, nox_channel. |
| **USER/blowback.h, blowback.c** | Two blowback valves; each sensor has the same register set in its block; **BLOW_CONTROL(ch, state)**. |
| **USER/calibration.h, calibration.c** | NOx/O₂ calibration per sensor (each sensor's cal trigger in its block), slope/intercept. |
| **USER/alarm.h, alarm.c** | Alarm thresholds and comparison with current output. |
| **USER/J1939.H, J1939.c** | J1939/CAN RX queue (with channel index), TX heater command. |
| **USER/modbus_slave.h, modbus_slave.c** | Modbus slave, register map (common + S1/S2 blocks), Flash read/write (24 floats). |
| **USER/modbus_flash.h, modbus_flash.c** | Internal Flash save/load for S1 and S2 calibration params (24 floats). |
| **USER/modbus_host.h, modbus_host.c** | Modbus master, 4–20 mA write/read. |
| **USER/modbus.h, modbus.c** | CRC, RS485 direction, common UART/timer macros. |
| **Core/** | HAL init, main, FreeRTOS, clock, interrupts. |
| **HARDWARE/** | OLED, font, SD card (FATFS, config). |
| **FATFS/** | FatFS and user_diskio. |
| **MDK-ARM/** | Keil project and startup. |

---

## 8. Version and Toolchain

- **IDE:** Keil µVision (MDK-ARM).  
- **Target:** STM32F103RCTx.  
- **HAL:** STM32Cube FW_F1.  
- **RTOS:** FreeRTOS with CMSIS-RTOS v2.  
- Generated from **STM32CubeMX** (NOx_RCT6.ioc); re-generate only if you change device or peripherals, and re-apply any manual edits under USER and application code.

---

## 9. Safety and Compliance

- This is a **debug/monitor** system. Use it in accordance with your site safety and emissions regulations.
- Relay and valve wiring must match the intended blowback and calibration plumbing; verify before applying power or gas.

---

*NOxDebug – NOx sensor monitor and debug system. Supports single or dual sensors (SA 0x52 / 0x51) with configurable work mode (single with channel select, primary-backup, fusion) and per-sensor calibration. Default mode is primary-backup. Common Modbus registers: NOx/O₂ output, output status, 4–20 mA, mode, alarm; sensor registers are symmetric (same count and layout for both). For register details and source-level behaviour, see the code and comments in USER/ and app_config.h.*
