# EV ADAS — Build & Run Guide

This is the companion guide to the firmware in `Core/` and the dashboard in
`Dashboard/`. It starts from a completely empty STM32CubeIDE workspace and
ends with a running simulation talking to the Python dashboard.

Target: **STM32F103C8T6 ("Blue Pill")**, simulated in **PICSimLab**, built
with **STM32CubeIDE**.

---

## 0. Before you start

You should already have (per the course's Day-1/Day-2 installation material):
- STM32CubeIDE installed
- PICSimLab installed, with the Blue Pill board support
- Python 3.9+ installed

Install the dashboard's Python dependencies:
```bash
pip install -r Dashboard/requirements.txt
```

---

## 1. Create the CubeIDE project

1. **File → New → STM32 Project.**
2. **Board/MCU** tab → search **STM32F103C8Tx** → Next.
3. Project name `EVADAS_Firmware` → **Targeted Language: C** → Finish → "Initialize peripherals with their default mode?" → **No** (we'll configure everything by hand below so the peripheral names match the code exactly).

This opens the `.ioc` pinout editor. Everything in section 2 happens there.

---

## 2. CubeMX configuration (the .ioc)

### 2.1 Clock configuration (`Clock Configuration` tab)
The Blue Pill has an 8 MHz HSE crystal, so:
1. `RCC` (left panel, `System Core`) → **High Speed Clock (HSE): Crystal/Ceramic Resonator**.
2. Go to the **Clock Configuration** tab. Set **Input frequency = 8**. Click on
   the **PLL Source Mux → HSE**, **PLLMul = x9**, and set **System Clock
   Mux → PLLCLK**. CubeMX will auto-fill the tree; confirm you land on:
   - SYSCLK = **72 MHz**
   - APB1 Prescaler = **/2** → APB1 = 36 MHz (timer clock = ×2 = **72 MHz**)
   - APB2 Prescaler = **/1** → APB2 = 72 MHz
   - ADC Prescaler = **/6** → ADC clock = **12 MHz** (must stay ≤ 14 MHz)
   - Flash latency: CubeMX will set 2 wait states automatically.

### 2.2 GPIO — set these exact "User Labels"
Click each pin on the chip diagram, set **Mode**, then type the **User
Label** exactly as shown (right-click the pin → "Enter User Label", or use
the Pinout view's label field). The label is what makes CubeMX generate the
`TRIG_FRONT_Pin` / `LED_FAULT_GPIO_Port`-style macros the code expects.

| Pin  | Mode                        | User Label        |
|------|-----------------------------|--------------------|
| PB0  | GPIO_Output                 | `TRIG_FRONT`       |
| PB1  | GPIO_Input                  | `ECHO_FRONT`       |
| PB2  | GPIO_Output                 | `TRIG_LEFT`        |
| PB3  | GPIO_Input                  | `ECHO_LEFT`        |
| PB4  | GPIO_Output                 | `TRIG_RIGHT`       |
| PB5  | GPIO_Input                  | `ECHO_RIGHT`       |
| PB8  | GPIO_Output                 | `LED_COLLISION`    |
| PB9  | GPIO_Output                 | `LED_BLINDSPOT_L`  |
| PB10 | GPIO_Output                 | `LED_BLINDSPOT_R`  |
| PB11 | GPIO_Output                 | `LED_FAULT`        |

For the three ECHO pins, open **GPIO Settings** (bottom panel) and set
**Pull-up/Pull-down = Pull-down** (keeps the line at a defined LOW when the
simulator's virtual sensor is idle/disconnected, instead of floating).

### 2.3 ADC1 — 4 channels, single conversion, no DMA
`Analog → ADC1`:
- Enable **IN0**, **IN1**, **IN2**, **IN3** (these are PA0–PA3).
- **Mode**: Scan Conversion Mode = **Disabled**, Continuous Conversion Mode = **Disabled**.
- **Configuration → Rank**: only Channel 0 needs a rank (Rank 1) — the firmware reconfigures the active channel itself before every read (see `ReadAdcChannel()` in `main.c`), so you don't need to set up all four here.
- Sampling Time: 55.5 Cycles.
- **No DMA.** Don't add anything under System Core → DMA for ADC1.

> **Why not Continuous + DMA (the "normal" efficient way):** PICSimLab's
> simulated ADC only implements single-shot conversions — configuring
> Continuous mode makes the simulator itself hard-crash with `qemu:
> hardware error: Mode Single conversion is only implemented`. Each of the
> four channels is instead read with its own quick start → poll → stop
> cycle, every 10 ms. This is a simulator limitation, not something wrong
> with continuous mode on real silicon — if you ever flash this to an
> actual Blue Pill, continuous+DMA would work fine there.

### 2.4 TIM1 — 20 kHz motor PWM (PA8)
`Timers → TIM1`:
- Clock Source = **Internal Clock**.
- **Channel1 = PWM Generation CH1**.
- Parameter Settings: **Prescaler = 0**, **Counter Period (ARR) = 3599** (72 MHz / 3600 = 20 kHz). Pulse = 0.
- Leave the Break/Dead-Time defaults (Break disabled) — CubeMX still generates that struct because TIM1 is an advanced-control timer; you don't need to change anything there.

> **Why TIM1 only does PWM here, with no 10 ms interrupt:** an earlier draft
> of the schedule has TIM1 doing double duty (a 10 ms base-timer interrupt
> *and* 20 kHz PWM on the same channel). Those can't coexist — a timer's
> Auto-Reload Register sets one frequency for the whole timer, so it can't
> be both "period = 10 ms" and "period = 1/20000 s" simultaneously. Since
> PA8 is hard-wired to TIM1_CH1 (no other timer can reach that pin), TIM1
> is dedicated entirely to the 20 kHz PWM, and the 10 ms EV-control tick is
> instead driven from `HAL_GetTick()` in the main loop (a standard SysTick-based
> cooperative scheduler — see `main.c`). Functionally you still get an
> accurate 10 ms(ish) tick; it's just not a hardware timer interrupt.

### 2.5 TIM2 — free-running 1 MHz counter (HC-SR04 timebase)
`Timers → TIM2`:
- Clock Source = **Internal Clock**.
- **Prescaler = 71**, **Counter Period = 65535**. No channels, no interrupt needed — it's just a free-running microsecond counter that `ultrasonic.c` reads directly.

### 2.6 TIM3 — 100 ms scheduler tick
`Timers → TIM3`:
- Clock Source = **Internal Clock**.
- **Prescaler = 7199**, **Counter Period = 999** (→ 100 ms).
- **NVIC Settings** tab (still within TIM3) → check **TIM3 global interrupt**.

### 2.7 TIM4 — buzzer PWM (PB6)
`Timers → TIM4`:
- Clock Source = **Internal Clock**.
- **Channel1 = PWM Generation CH1**.
- **Prescaler = 71**, **Counter Period = 499** (a ~2 kHz default tone; the firmware rewrites this at runtime per alarm level, see `Buzzer_SetTone()` in `main.c`).

### 2.8 USART1 — 115200 8N1, blocking TX, interrupt RX, no DMA
`Connectivity → USART1`:
- Mode = **Asynchronous**.
- Baud Rate = **115200**, Word Length = 8 Bits, Parity = None, Stop Bits = 1.
- **NVIC Settings** → check **USART1 global interrupt**.
- **No DMA.** Don't add anything under System Core → DMA for USART1 either.

> **Why not DMA here too:** PICSimLab's own maintainer has said the
> qemu-based STM32 backend's peripheral models are incomplete and "crash a
> lot" — DMA is one of the more complex peripherals to emulate correctly,
> and having just hit a confirmed crash on the ADC side, it's not worth
> gambling on it for USART too. Telemetry frames are sent with a plain
> blocking `HAL_UART_Transmit()` instead. A ~100-byte frame takes about
> 9 ms at 115200 baud; since that only happens from the 100 ms scheduler
> tick (not the time-critical 10 ms EV loop), and that loop already uses
> variable-dt integration to absorb bigger stalls than this from the
> HC-SR04 polling, the blocking call costs nothing functionally. RX stays
> interrupt-driven (not DMA), which is a far more basic mechanism and much
> more likely to be solid.

### 2.9 Project Manager tab
- **Code Generator**: leave **"Generate peripheral initialization as a pair of '.c/.h' files per peripheral"** *unchecked* (we want everything in the single `main.c`, matching the code in this bundle).
- Click **GENERATE CODE**.

---

## 3. Drop in the application code

CubeMX has now created a project with correct, matching `MX_xxx_Init()`
functions. From this bundle, copy in:

1. **Replace** the generated `Core/Src/main.c` with the one in this bundle
   — it contains the same `MX_xxx_Init()` bodies CubeMX just generated for
   you (so nothing is lost) *plus* the application logic in the
   `USER CODE` sections. If your CubeMX output differs in any small way
   (a different Prescaler, etc.), trust your own `.ioc` — just make sure
   the `USER CODE` sections match what's in this file.
2. **Copy as-is** into `Core/Inc/` and `Core/Src/`: `ultrasonic.h/.c`,
   `ev_control.h/.c`, `adas.h/.c`, `fault.h/.c`, `uart_shell.h/.c`. These
   don't touch any CubeMX-generated file, so they can't conflict.
3. Leave `stm32f1xx_hal_msp.c`, `stm32f1xx_it.c`, `main.h`, and
   `stm32f1xx_hal_conf.h` as CubeMX generated them — the copies in this
   bundle are included only so you can **diff against your own output** if
   something doesn't build; they should already match.

### 3.1 One printf gotcha (doesn't affect this project, but you'll hit it eventually)
STM32CubeIDE links against **newlib-nano** by default, which strips
floating-point support out of `printf`/`snprintf` unless you either tick
**Project Properties → C/C++ Build → Settings → MCU Settings → "Use float
with printf from newlib-nano"**, or add `-u _printf_float` under **MCU GCC
Linker → Miscellaneous → Other flags**. Otherwise every `%f` silently
prints garbage or `0.000`. This firmware sidesteps the issue entirely — the
telemetry formatter (`FormatFixed()` in `uart_shell.c`) does fixed-point
formatting by hand instead of calling `printf("%f", ...)`, which also saves
several KB of flash on a 64 KB part — but it's worth knowing about for your
own debug prints elsewhere.

### 3.2 Build
**Project → Build All** (or the hammer icon). Expected result: 0 errors.
Every file in this bundle was cross-compiled against the real
STM32F103xB CMSIS headers and STM32F1 HAL driver with
`-mcpu=cortex-m3 -mthumb -Wall -Wextra -Werror` and linked into a complete
ELF against the real `STM32F103XB_FLASH.ld`/startup file before being
handed to you (40.1 KB flash / 3.3 KB RAM — comfortably inside the
64 KB/20 KB budget), so if your build fails, it's almost always a mismatch
between your `.ioc` clicks and section 2 above (check pin/timer/DMA
assignments first).

---

## 4. PICSimLab setup

1. Open PICSimLab → **Board → Blue Pill (bluepill)**.
2. **File → Load firmware (hex/bin)** → point it at the `.bin` your CubeIDE
   build produced (`Debug/EVADAS_Firmware.bin`, or whichever build config
   you used).
3. Wire up the simulated peripherals to match section 2.2/2.3's pin map:
   - 4× potentiometer components on **PA0, PA1, PA2, PA3** (accelerator,
     brake, SOC override, motor-temp override).
   - 3× HC-SR04 components: TRIG/ECHO pairs on **PB0/PB1** (front),
     **PB2/PB3** (left), **PB4/PB5** (right).
   - 4× LEDs on **PB8, PB9, PB10, PB11**.
   - A buzzer (or just an oscilloscope probe, if PICSimLab's buzzer part
     doesn't do PWM tone) on **PB6**.
   - The built-in **UART Terminal** component on **PA9/PA10** at **115200
     8N1** — this alone is enough to *watch* the `$EV`/`$AD` frames
     scrolling by and to type shell commands (see section 6) directly, no
     Python required, which is the fastest way to sanity-check the build.
4. For the Python dashboard to receive the same stream, you need a virtual
   serial port bridge — **VSPE** (Virtual Serial Ports Emulator) is what the
   course material links for this: create a paired port (e.g. `COM10 ↔
   COM11`), point PICSimLab's UART at one end, and `--port COM11` (or
   whichever) at the dashboard. On Linux, `socat -d -d PTY,raw,echo=0
   PTY,raw,echo=0` gives you the equivalent pair of `/dev/pts/N` devices.
5. **Run** the simulation (▶).

---

## 5. Run the dashboard

```bash
# against the simulator (via VSPE/socat, see above)
python Dashboard/dashboard.py --port COM11 --baud 115200

# or, with no hardware/simulator running at all, to see it work immediately:
python Dashboard/dashboard.py --demo
```

---

## 6. Wire protocol reference

Every telemetry line is human-readable ASCII wrapped in a CRC-checked
envelope, so it's directly viewable in PICSimLab's UART Terminal *and*
machine-parseable by the dashboard:

```
$TAG,field:value,field:value,...*CCCC\r\n
```
`CCCC` is a 4-hex-digit **CRC-16/CCITT** (poly `0x1021`, init `0xFFFF`) over
the bytes between `$` and `*`. `uart_shell.c`'s `CRC16_CCITT()` and
`dashboard.py`'s `crc16_ccitt()` were built and numerically cross-checked
against each other (see the design notes below) — a corrupted frame is
dropped rather than displayed with wrong values.

| Tag    | Sent                     | Fields |
|--------|--------------------------|--------|
| `$EV`  | every 100 ms             | `SPD` km/h, `SOC` %, `TRQ` Nm, `PWR` kW, `RNG` km, `TMP` °C, `MODE` (0=ECO/1=NORMAL/2=SPORT), `ACC` %, `BRK` %, `UP` ms uptime |
| `$AD`  | every 100 ms             | `F`/`L`/`R` cm, `TTC` s (`--` if n/a), `COL` (0/1/2 = none/warning/critical), `BSD` two bits (left, right), `ALM` (0-3), `FLT` fault byte (hex) |
| `$ST`  | on vehicle-state change | `STATE` (0=PARKED,1=READY,2=DRIVING,3=REGEN,4=FAULT), `MODE`, `FLT` |
| `$ACK` | on the `status` command | `UP`, `VER` |

### Shell commands (type into PICSimLab's UART Terminal, or any serial monitor)
All commands are lowercase, one per line:

```
mode eco | mode normal | mode sport        set drive mode
speed set <kmh>  (or just: speed <kmh>)     override speed (bypasses physics)
speed auto                                  return speed to the physics model
soc set <pct>    (or just: soc <pct>)       override battery %
soc auto                                    return SOC to the energy model
obstacle <cm>                               inject a front-obstacle reading
blindspot on | blindspot off                force both blind-spot flags
fault inject motor|soc|col|sen|com          force that fault -> FAULT state
fault clear   (alias: reset)                clear all faults/overrides -> PARKED
status                                      request one $ACK frame
stream on | stream off                      enable/disable the 10 Hz frames
temp <celsius>                              override motor temperature (bonus)
```

---

## 7. Design decisions worth knowing about

The requirement doc, the 14-day schedule, and the dashboard presentation
disagree with each other in a few small places (normal for a multi-document
spec written over time). Here's what was chosen and why, so you can explain
it if asked:

- **No DMA anywhere in the design, on either ADC1 or USART1.** The first
  version of this firmware used Continuous+Scan+DMA for the ADC (the
  efficient, idiomatic way to do it on real hardware) and DMA for the
  USART1 telemetry TX. PICSimLab's simulated ADC crashes outright on
  anything but single-conversion mode (`qemu: hardware error: Mode Single
  conversion is only implemented`), and its maintainer has said openly
  that the qemu-based STM32 backend's peripheral models are incomplete.
  Both peripherals were moved to the simplest mechanism available — single-
  shot polled ADC reads, and blocking UART transmit — since those are far
  more likely to be solid in an emulator this basic. If you ever flash
  this firmware onto a **real** Blue Pill, both of the original DMA-based
  approaches are legitimate, more CPU-efficient options worth switching
  back to; they just don't survive this particular simulator.
- **ASCII frames instead of raw binary packets.** The requirements doc
  describes "structured binary packets"; the presentation deck shows plain
  `SPD:72.5 SOC:79.3 ...`-style text. ASCII was used because (a) it's what
  PICSimLab's built-in UART Terminal can actually display without a hex
  viewer, (b) it's what the schedule's own Python homework ("parse_line()
  regex") assumes, and (c) the `$TAG,...*CRC` envelope still gives you the
  exact same CRC-16 packet-integrity guarantee the schedule's test cases
  (TC-13) ask for — you get both readability and integrity checking.
- **Buzzer on PB6 (TIM4_CH1), not PB9.** The presentation lists the buzzer
  on PB9, but the requirements doc's own pin table already assigns PB9 to
  the left blind-spot LED. The schedule's Day-5 entry says "TIM4 PWM CH1",
  which maps to PB6 on this chip — that's a free pin with no conflict, so
  that's what's used.
- **TIM1 is PWM-only; the 10 ms EV tick runs off SysTick.** See section 2.4
  above for the full reasoning.
- **`FAULT_SEN`/`FAULT_COM` are logged but don't force `STATE_FAULT` on
  their own.** The bit-field register tracks all five conditions, but the
  design doc's own "Auto-Fault Logic" section only lists motor-overtemp,
  critical-SOC, and collision-critical as triggering a hard trip; a sensor
  or comms hiccup instead "degrades gracefully" (matches the NFR table
  literally). A manually-injected fault of *any* type, via the `fault
  inject` shell command, still forces `STATE_FAULT`, since that command's
  whole purpose is to let you exercise the recovery workflow on demand.
- **The SOC potentiometer (PA2) is a live manual override, not the primary
  SOC source.** The primary model is the energy-integration calculation in
  `ev_control.c` (an explicit "High" priority requirement). Touching the
  potentiometer more than ~1% from its last reading hands control to it
  immediately — handy for demoing "what happens at 5% battery" without
  waiting for the model to drain there.
- **Two small state-machine transitions were added** (`READY → PARKED`
  when the accelerator is released, and `REGEN → READY` vs. `REGEN →
  DRIVING` depending on whether the accelerator is still pressed) — the
  design doc's state table doesn't list what happens in those two cases,
  so these fill the gap in the direction that seemed safest/most obvious.

## 8. Mapping onto the 14-day test plan

The shell commands above cover the fault-injection and sensor/speed/SOC
overrides the later-day test cases (TC-08 through TC-17 in the schedule)
ask for directly — e.g. TC-13 (packet integrity) is exercised just by
watching the dashboard's "Packets bad" counter stay at 0 over a long run;
TC-15 (fault clear recovery) is `fault inject motor` then `fault clear`,
watching `$ST` fire twice.

---

## 9. What's included

```
Core/Inc/   main.h, stm32f1xx_hal_conf.h, stm32f1xx_it.h,
            ultrasonic.h, ev_control.h, adas.h, fault.h, uart_shell.h
Core/Src/   main.c, stm32f1xx_it.c, stm32f1xx_hal_msp.c,
            ultrasonic.c, ev_control.c, adas.c, fault.c, uart_shell.c
Dashboard/  dashboard.py, requirements.txt
README.md   (this file)
```
