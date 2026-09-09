# Autonomous Indoor Delivery Robot — Hardware & System Architecture

**Project:** RoboDog Indoor Delivery System
**Course context:** Computer Engineering capstone / project
**Main MCU:** ATmega1284P (standalone) — decision brain
**Co-processor:** Arduino Uno/Nano (ATmega328P) — sensor front-end
**Explicitly excluded:** Raspberry Pi, Jetson, LiDAR, SLAM, on-board deep learning, ROS
**Locomotion:** Differential-drive wheels (2 driven + 2 casters), autonomous, not RC

---

## 0. Reading guide & how this relates to the existing repo

This repo currently holds a **Python simulation** of the same product (A\* over a 20×30 grid, an
11-state FSM, package validation, QR verification, YOLOv8 perception). That code stays useful —
it becomes the **algorithm sandbox and test harness**. This document describes the **physical
robot** and the firmware architecture, and where the two must diverge.

Three decisions in the existing design cannot survive on an ATmega and are re-specified here:

| Existing (Python sim) | Problem on ATmega | Replacement in this architecture |
|---|---|---|
| QR code scan for sender/receiver | Requires a camera + image decoding — impossible on an AVR, and a camera is explicitly out of the core system | **6-digit delivery PIN** entered on the keypad. Same security role, zero extra hardware. |
| YOLOv8 person detection driving navigation | Needs Pi/GPU-class compute | Ultrasonic ring is the **primary** obstacle sensor. Optional ESP32-CAM contributes a *hint flag*, never a navigation input. |
| Package dimension entry (15×15×15 cm) validated in software | User can lie; no dimension sensor exists | **Physical cargo bay sized 16×16×16 cm internal.** If it fits, it's valid. Weight is the only measured constraint. |

Multi-floor navigation (stairs/elevator) stays **simulation-only**. The physical robot is
single-floor. This one scope cut removes the largest chunk of mechanical and safety risk.

---

## 1. System Block Diagram

```
                        ┌───────────────────────────────────────┐
                        │            USER (sender)              │
                        │      keypad + 20x4 LCD + buzzer       │
                        └───────────────┬───────────────────────┘
                                        │ I2C (0x27 LCD, 0x20 keypad expander)
                                        │
  ┌─────────────────────┐               │               ┌──────────────────────────┐
  │  LOAD CELL 5 kg     │  analog       │               │   MPU6050 (yaw gyro)     │
  │  + HX711 24-bit ADC ├───────────┐   │   ┌───────────┤   I2C, same bus as LCD   │
  └─────────────────────┘  2-wire   │   │   │           └──────────────────────────┘
                                    ▼   ▼   ▼
   ═══════════════════════════════════════════════════════════════════════════
   ║                  ATmega1284P  @16 MHz  —  MAIN MCU / BRAIN               ║
   ║                                                                          ║
   ║   Menu & LCD UI  │  PIN generate/verify  │  Package weight validation    ║
   ║   Predefined map (PROGMEM) │ A* / Dijkstra global planner                ║
   ║   Odometry (encoders + gyro fusion) │ Waypoint sequencer                 ║
   ║   Local avoidance arbitration │ Rerouting policy │ Master FSM            ║
   ║   Wheel velocity PID + heading PID │ Safety supervisor │ Watchdog        ║
   ═══════╤═══════════════╤══════════════════╤═════════════════╤═════════════╤═
         │ UART0          │ INT2             │ 4x PWM + EN     │ 4x PCINT    │ ADC
         │ 38400 8N1      │ SAFETY_STOP      │                 │             │
         │ (bidirectional)│ (active-low,     │                 │             │
         │                │  wired-OR)       │                 │             │
   ┌─────▼──────────────┐ │            ┌─────▼─────────────┐   │             │
   │  ARDUINO UNO/NANO  │ │            │ 2x BTS7960 (IBT-2)│   │             │
   │  ATmega328P @16MHz │ │            │  MOSFET H-bridge  │   │             │
   │  SENSOR FRONT-END  │ │            └─────┬───────┬─────┘   │             │
   │                    │ │                  │       │         │             │
   │ - 5x HC-SR04 ping  │ │            ┌─────▼───┐ ┌─▼───────┐ │             │
   │   scheduler        │ │            │ MOTOR L │ │ MOTOR R │ │             │
   │ - median filter    │ │            │ JGB37   │ │ JGB37   │ │             │
   │ - unit conversion  │ │            │ 12V gear│ │ 12V gear│ │             │
   │ - fault detection  │ │            └──┬───┬──┘ └──┬───┬──┘ │             │
   │ - packet framing   │ │               │   │hall   │   │hall│             │
   │ - EMERGENCY assert ├─┘               │   └───────┼───┴────┘             │
   └──┬──────────────┬──┘                 │           │      quadrature      │
      │              │ optional        ┌──▼───┐   ┌───▼──┐                   │
  ┌───▼──────────┐ ┌─▼─────────────┐   │WHEEL │   │WHEEL │                   │
  │ 5x HC-SR04   │ │ ESP32-CAM     │   │ 65mm │   │ 65mm │                   │
  │ FC FL FR L R │ │ (2-3 GPIO     │   └──────┘   └──────┘                   │
  └──────────────┘ │  flag lines)  │                                         │
                   └───────────────┘   ┌─────────────────────────────────┐   │
  ┌──────────────────────────┐         │ 2x BUMPER MICROSWITCH (front)   │   │
  │  BATTERY 3S Li-ion 11.1V ├────┐    └────────────┬────────────────────┘   │
  │  6.8 Ah + BMS + 10A fuse │    │                 │ wired-OR into INT2     │
  └──────────────────────────┘    │                 └────────────────────────┘
             │                    │
             │  ┌─────────────────▼──────────────────┐
             ├──┤ E-STOP (latching mushroom) ──► RELAY├──► motor power rail only
             │  └────────────────────────────────────┘    (logic stays alive)
             │
             └──► 2x LM2596 buck ──► 5V LOGIC RAIL ──► MCUs, LCD, sensors
                                                        ▲ VBAT sense to ADC0
```

**Two-tier reasoning.** The Arduino owns everything that is *timing-ugly but decision-free*
(five ultrasonic sensors that must be fired sequentially and each block for up to 30 ms). The
ATmega owns everything that is *decision-critical and must never be delayed* (motor loops,
odometry, FSM, safety). This keeps the brain's control loop jitter-free even though ranging is
inherently slow — which is the actual engineering reason for a two-MCU split, not just "because
we have two boards".

---

## 2. Exact role of ATmega vs Arduino

### 2.1 Why the main MCU is an ATmega1284P, not an ATmega328P

An Arduino Uno *is* an ATmega328P, so "ATmega vs Arduino" only means something if the main brain
is a **standalone chip on your own board**. That is the design here. But the 328P is the wrong
chip for the brain, for two measurable reasons:

**SRAM.** The 328P has 2048 bytes. A grid A\* over the existing 20×30 map needs, at minimum:

| Structure | Size on 600 cells |
|---|---|
| Dynamic obstacle overlay (1 bit/cell) | 75 B |
| Closed set (1 bit/cell) | 75 B |
| g-score (uint8, scaled cost) | 600 B |
| Came-from direction (4 bits/cell) | 300 B |
| Open-list binary heap (capped 128 × 3 B) | 384 B |
| **Total planner working set** | **~1.43 KB** |

That leaves ~600 B for the stack, LCD buffers, UART ring buffers, PID state and odometry on a
2 KB chip. It will work in a demo and fail under load — stack overflow on an AVR is silent and
looks like random resets. The ATmega1284P has **16 KB SRAM**: the same planner uses 9% of RAM.

**UART count.** The 328P has one USART. You need it for the Arduino link, which means you have
**no serial debug channel** on the exact device whose decision logic you most need to observe.
The 1284P has two USARTs: USART0 for the sensor link, USART1 for a permanent USB-TTL debug port.

The 1284P is also **PDIP-40** — hand-solderable, breadboardable, socketable by a student, unlike
the ATmega2560 (TQFP-100). Flash is 128 KB vs 32 KB, so map tables, string tables and a full
menu tree fit without PROGMEM gymnastics.

> **Fallback if you must use ATmega328P:** drop the grid planner and use the waypoint-graph
> planner only (§7.1) — ~40 nodes, under 400 B — and give up serial debug or bit-bang a
> transmit-only debug line. Document the compromise. Do not try to run a 600-cell grid A\* on it.

### 2.2 ATmega1284P — the brain (owns all decisions and all actuators)

| Responsibility | Rate | Notes |
|---|---|---|
| Master FSM (§8) | 50 Hz | Single source of truth for robot behaviour |
| LCD menu rendering | 10 Hz, partial updates | I2C LCD is slow; never full-clear in a loop |
| Keypad event handling | event-driven via expander INT | Destination entry, PIN entry, confirm/cancel |
| Package weight read (HX711) | 10 Hz | Tare at IDLE, stability check before accept |
| Package accept/reject decision | on event | Weight limit + stability + bay-closed |
| Delivery PIN generate & verify | on event | Replaces QR; PRNG seeded from ADC noise + timer |
| Global path planning | on demand, <30 ms | A\* over PROGMEM map (§7) |
| Rerouting policy | on blockage | Wait → sidestep → re-plan → abort ladder |
| Odometry integration | 100 Hz | Encoder quadrature + gyro yaw fusion |
| Waypoint sequencing & junction re-localisation | 20 Hz | Corrects dead-reckoning drift |
| Local avoidance arbitration | 50 Hz | Consumes Arduino telemetry, outputs speed scale |
| Wheel velocity PID (×2) | 100 Hz | Closed loop on encoder counts |
| Heading PID | 50 Hz | Closed loop on fused yaw |
| Motor PWM generation | 7.81 kHz carrier | Timer1 + Timer2, 8-bit, matched |
| Safety supervisor | 50 Hz + INT2 | Link timeout, bumper, over-current, brownout |
| Battery monitoring & low-battery return | 2 Hz | ADC0 divider |
| Watchdog | WDT 1 s | Reset on lock-up, log reason |

**The ATmega is the only device that may command motion.** No exceptions. This is what makes the
robot autonomous-by-construction: there is no path from any input device to the motors that does
not pass through the ATmega's FSM.

### 2.3 Arduino Uno/Nano — the sensor front-end (owns no decisions)

| Responsibility | Rate | Notes |
|---|---|---|
| Fire 5 ultrasonic sensors in a crosstalk-safe schedule | 5 Hz full sweep | §5.4 |
| Interrupt-driven echo capture (all echoes on PORTB, one PCINT vector) | — | Never use blocking `pulseIn()` |
| 3-sample median filter per sensor | per sweep | Kills the single-ping fliers HC-SR04 is famous for |
| Convert to millimetres, clamp to valid range | per sweep | `0xFFFF` = no echo, `0x0000` = sensor fault |
| Per-sensor fault detection (stuck-high, no-echo streak, out-of-family) | per sweep | Reported in flags byte |
| Frame and transmit telemetry packet | 20 Hz | §5 protocol |
| Assert hardware `EMERGENCY_OUT` line | <10 ms | If any front sensor < 200 mm — bypasses the serial link entirely |
| Latch optional vision flag lines from ESP32-CAM | 20 Hz | Passed through as 2 bits, never interpreted |
| Heartbeat LED | 1 Hz | On-robot liveness without a laptop |

**What the Arduino must never do:** decide to stop, decide to turn, plan, know the map, know the
destination, know the package, or touch a motor pin. It reports the world in millimetres. That's
it. This is why the interface between the two chips is stable enough to freeze in week 2 (§11).

### 2.4 Interaction contract

* Arduino → ATmega: `SENSOR_TELEMETRY` @ 20 Hz, unsolicited, plus fault reports.
* ATmega → Arduino: mode/rate commands and a 1 Hz heartbeat.
* **Two independent safety paths:** the serial packet stream (rich, 50 ms latency) and the
  `SAFETY_STOP` hardware line into INT2 (1 bit, <10 ms, works even if firmware on either side
  has crashed). The bumper microswitches are wired into the *same* line, so mechanical contact
  stops the robot with no software in the loop at all.
* If the ATmega does not receive a valid telemetry packet within **250 ms**, it enters
  `SAFE_STOP`. Blind driving is never permitted.

---

## 3. Bill of Materials

Prices are indicative USD for Indonesian marketplaces (Tokopedia/Shopee/Digiware), Sept 2026.
Verify locally — treat the *ratios* as the useful information, not the absolute numbers.

### 3.1 Compute & programming

| # | Component | Qty | ~Unit | ~Total | Why this part is needed |
|---|---|---|---|---|---|
| 1 | **ATmega1284P-PU** (PDIP-40) | 2 | $6.00 | $12.00 | Main brain. 16 KB SRAM for the planner, 2 USARTs, DIP package. Buy 2 — one spare, because a wrong fuse setting bricks a chip. |
| 2 | 40-pin DIP socket | 2 | $0.30 | $0.60 | Never solder the MCU directly. Lets you swap the spare in and program off-board. |
| 3 | 16.000 MHz crystal + 2× 22 pF ceramic | 2 sets | $0.40 | $0.80 | External clock. Required for the 0.2% UART error figure (§5.2); the internal 8 MHz RC oscillator drifts ±10% and will corrupt the link. |
| 4 | **Arduino Uno R3** (sensor node) | 1 | $8.00 | $8.00 | Sensor front-end. Uno over Nano for the first build: full-size headers survive re-wiring. Swap to a Nano for the final chassis to save space. |
| 5 | Arduino Uno R3 (second board) | 1 | $8.00 | $8.00 | Dual purpose: temporary "brain" in Phase 3 (§11) so you get a driving robot before touching a bare chip, then permanently your **Arduino-as-ISP** programmer for the 1284P. |
| 6 | USB-TTL adapter (CP2102 / FTDI) | 1 | $3.00 | $3.00 | Permanent debug console on the 1284P's USART1. This is the single highest-value $3 in the project. |
| 7 | 10 kΩ resistor, 100 nF cap (reset circuit) | 1 set | $0.20 | $0.20 | Reset pull-up + ISP auto-reset. |
| | | | | **$32.60** | |

### 3.2 Sensing

| # | Component | Qty | ~Unit | ~Total | Why this part is needed |
|---|---|---|---|---|---|
| 8 | **HC-SR04** ultrasonic module | 6 | $1.50 | $9.00 | 5 in service (front-centre, front-left 45°, front-right 45°, left 90°, right 90°) + 1 spare. Front trio covers the robot's full width; side pair does corridor centring *and* junction detection, which is what makes drift correction possible (§7.3). |
| 9 | **MPU6050 (GY-521)** 6-axis IMU | 1 | $2.00 | $2.00 | Yaw rate only. A $2 part that turns "roughly 90°" into "90° ±2°". Encoder-only turns accumulate 5–15° error per turn on tile floors due to wheel slip; that error is unrecoverable. Strongly recommended despite the minimalism rule. |
| 10 | **Load cell, single-point, 5 kg** (TAL220B or equivalent) | 1 | $8.00 | $8.00 | Measures package weight — the one hard acceptance criterion in the spec. 5 kg range gives headroom over the 2 kg limit plus the platform tare. Single-point type is designed to carry a flat plate, so one is enough — no 4-cell bathroom-scale hack. |
| 11 | **HX711** load-cell amplifier module | 1 | $1.50 | $1.50 | 24-bit ADC + instrumentation amp. The AVR's 10-bit ADC cannot read a load cell's ~1 mV/V output; this is not optional if you want grams. |
| 12 | Lever microswitch, SPDT, roller arm | 2 | $0.50 | $1.00 | Front bumper. Last-resort contact detection for the things ultrasound cannot see: glass, chair legs, dark fabric, angled walls. Wired straight into the hardware stop line — no firmware involved. |
| 13 | Wheel encoders | — | — | included | Comes integrated in item 17 (hall-effect). Do not buy separate optical encoders. |
| | | | | **$21.50** | |

### 3.3 Human interface

| # | Component | Qty | ~Unit | ~Total | Why this part is needed |
|---|---|---|---|---|---|
| 14 | **LCD 2004 (20×4)** + PCF8574 I2C backpack | 1 | $5.00 | $5.00 | 20×4 not 16×2: you need to show destination, weight, state and a PIN simultaneously. I2C backpack reduces 6 wires to 2. |
| 15 | **4×4 membrane keypad** | 1 | $2.00 | $2.00 | Direct room-number entry (`201#`) plus 6-digit PIN entry. A 5-button scheme forces users to scroll a room list — worse UX and more menu code. |
| 16 | **PCF8574** I2C expander module | 1 | $1.00 | $1.00 | Puts the keypad's 8 lines on the existing I2C bus. Frees 7 GPIO on the ATmega for $1 (§4.1 shows the pin budget is genuinely that tight). |
| 17 | Active piezo buzzer, 5 V | 1 | $0.50 | $0.50 | Arrival alert, rejection beep, and moving-robot audible warning. A silent moving robot in a corridor is a safety complaint waiting to happen. |
| 18 | 5 mm LED (green/red/blue) + 220 Ω | 3 | $0.15 | $0.45 | On-robot state indication without a laptop. |
| 19 | **E-stop, latching mushroom, 22 mm, NC** | 1 | $4.00 | $4.00 | Mandatory. Breaks the motor power rail via the relay. Non-negotiable for any demo where the robot moves near people or a lecturer. |
| 20 | Rocker switch (logic / motor) | 2 | $0.50 | $1.00 | Independent logic and motor power. Lets you debug the brain with wheels dead. |
| | | | | **$13.95** | |

### 3.4 Actuation & drivetrain

| # | Component | Qty | ~Unit | ~Total | Why this part is needed |
|---|---|---|---|---|---|
| 21 | **JGB37-520 12 V DC gear motor, ~178 RPM, with hall encoder** | 2 | $14.00 | $28.00 | Two driven wheels = differential drive. Encoder is built in. 178 RPM on a 65 mm wheel gives 0.6 m/s top speed, so 0.3 m/s cruise sits at 50% duty — motors never run at their weak end. Torque check in §3.7. |
| 22 | Rubber wheel 65 mm with 6 mm hub coupling | 2 | $4.00 | $8.00 | Rubber, not plastic: slip is the direct cause of odometry drift. 65 mm keeps the geometry maths clean (204.2 mm circumference). |
| 23 | Ball caster / swivel caster 1" | 2 | $2.00 | $4.00 | Two casters (front and rear) make the platform statically stable so the cargo bay and load cell don't rock. One caster = tripod = the load cell reads garbage during acceleration. |
| 24 | **BTS7960 (IBT-2) 43 A H-bridge module** | 2 | $6.00 | $12.00 | One per motor. MOSFET-based: ~0.02 Ω on-resistance vs the L298N's ~2 V saturation drop. See §3.6 for why the L298N is the wrong final choice. Handles a 3 A stall without a heatsink. |
| 25 | Motor mounting bracket (37 mm) | 2 | $1.50 | $3.00 | Rigid mounting. Flexing brackets cause toe-in changes and curved "straight" driving. |
| | | | | **$55.00** | |

### 3.5 Power

| # | Component | Qty | ~Unit | ~Total | Why this part is needed |
|---|---|---|---|---|---|
| 26 | **3S Li-ion 18650 pack, 11.1 V 6.8 Ah, with BMS** | 1 | $28.00 | $28.00 | 11.1 V matches 12 V motors well. Li-ion + BMS over bare LiPo: over-discharge and short protection built in, far safer for a student team and for storage between demo days. ~2.5–3 h of driving (§9.3). |
| 27 | Charger 12.6 V 2 A (or 3S balance charger) | 1 | $8.00 | $8.00 | Matched charger. Charge outside the robot. |
| 28 | **LM2596 / MP1584 buck module, 5 V 3 A** | 2 | $2.00 | $4.00 | Switching regulator for the 5 V logic rail. One in service, one spare. §9.2 explains why the Arduino's onboard linear regulator cannot do this job. |
| 29 | Blade fuse holder + 10 A fuse (pack) | 1 | $1.50 | $1.50 | Between battery + and everything. A shorted motor lead without a fuse is a fire. |
| 30 | Power relay, SPST 12 V 30 A (automotive) | 1 | $2.00 | $2.00 | E-stop and MCU-commanded motor-rail cut. Logic keeps running so the LCD can say *why* it stopped. |
| 31 | 1000 µF 25 V electrolytic | 2 | $0.30 | $0.60 | Bulk decoupling at each motor driver's VBAT input. Absorbs commutation current spikes. |
| 32 | 100 nF ceramic | 20 | $0.05 | $1.00 | Per-IC decoupling + 3 across each motor's terminals/case (the standard fix for motor EMI resetting an MCU). |
| 33 | XT60 connector pair | 2 | $1.00 | $2.00 | Keyed, high-current battery connector. Do not use a barrel jack or Dupont for motor current. |
| 34 | Silicone wire 18 AWG (power) + 22 AWG (signal), 5 m each | 1 set | $6.00 | $6.00 | 18 AWG for the motor/battery rail, 22 AWG for signals. |
| | | | | **$53.10** | |

### 3.6 Structure, wiring & consumables

| # | Component | Qty | ~Unit | ~Total | Why this part is needed |
|---|---|---|---|---|---|
| 35 | Acrylic/aluminium chassis plate 300×200 mm | 2 | $6.00 | $12.00 | Two-deck layout: drivetrain + battery below, electronics + cargo above. Keeps motor wiring physically away from signal wiring. |
| 36 | M3 standoff / screw / nut kit | 1 | $6.00 | $6.00 | Deck separation and board mounting. |
| 37 | **Cargo bay box, 16×16×16 cm internal** (acrylic or 3D print) | 1 | $8.00 | $8.00 | This *is* the dimension validator. A package that physically fits is dimensionally valid — no sensor, no user-entered numbers, no lying. Mounts on the load cell plate. |
| 38 | Perfboard 8×12 cm (or 2-layer custom PCB) | 2 | $2.00 | $4.00 | One board for the ATmega brain, one for power distribution. A $10 JLCPCB order is worth it if your timeline allows. |
| 39 | Header pins, JST-XH 2/3/4-pin sets, Dupont jumpers | 1 set | $6.00 | $6.00 | **Every sensor on a connector, never soldered inline.** This is what makes the design modular and upgradeable. |
| 40 | Heatshrink, zip ties, spiral wrap, double-sided tape | 1 set | $6.00 | $6.00 | Cable management. Loose wires in a moving robot fail intermittently, which is the worst failure mode to debug. |
| 41 | 2.000 kg calibration weight (or known mass) | 1 | $5.00 | $5.00 | Load cell calibration and the 2 kg boundary test. Use it in every acceptance test. |
| | | | | **$47.00** | |

### 3.7 Optional / experimental (do not buy in the first order)

| # | Component | Qty | ~Unit | ~Total | Condition for buying |
|---|---|---|---|---|---|
| 42 | ESP32-CAM + programmer | 1 | $9.00 | $9.00 | Only after the ultrasonic robot completes 20 consecutive successful deliveries. Contributes 2–3 GPIO *hint* lines, never a navigation input (§4.5). |
| 43 | IR reflectance sensor (TCRT5000) for cliff/edge | 2 | $1.00 | $2.00 | Only if the route has an unguarded drop (stair landing, loading dock). Single-floor flat-corridor routes don't need it. |
| 44 | Photo-interrupter / IR beam for bay over-height | 1 | $1.50 | $1.50 | Only if users repeatedly leave the lid open. The lid microswitch is usually enough. |
| | | | | **$12.50** | |

### 3.8 Cost summary

| Group | Subtotal |
|---|---|
| Compute & programming | $32.60 |
| Sensing | $21.50 |
| Human interface | $13.95 |
| Actuation & drivetrain | $55.00 |
| Power | $53.10 |
| Structure & consumables | $47.00 |
| **Core build total** | **≈ $223** (≈ IDR 3.6 M) |
| Optional add-ons (later) | +$12.50 |
| **Recommended budget incl. 15% breakage/re-order** | **≈ $260** (≈ IDR 4.2 M) |

**Ultra-budget variant (~$140), and what it costs you:**

| Substitution | Saving | What you lose |
|---|---|---|
| 12 V 7 Ah SLA battery instead of Li-ion pack | −$14 | +2.3 kg mass → needs the full motor torque margin; runtime drops; voltage sags harder under stall |
| L298N instead of 2× BTS7960 | −$10 | ~2 V drop = ~20% of your motor voltage burned as heat; needs a heatsink; dies on repeated stalls; no current sense |
| "Yellow TT" gear motors, no encoders | −$25 | **No odometry at all** → no closed-loop distance, no waypoint navigation. Only acceptable in Phase 2/3 bench testing. |
| 16×2 LCD instead of 20×4 | −$2 | Cramped UI, more menu paging code |
| Skip MPU6050 | −$2 | Turn error 5–15° each, compounding. Bad trade. |

Cut the battery and the LCD if you must. **Do not cut the encoders, the MPU6050, the bumper, or
the E-stop** — each of those removes a capability or a safety property that cannot be recovered
in software.

### 3.9 Drivetrain sizing check (why these motors, with numbers)

```
Robot mass:            chassis+battery+electronics  ≈ 3.0 kg
                       max package                  ≈ 2.0 kg
                       total m                      = 5.0 kg

Rolling resistance (indoor tile/low carpet, Crr ≈ 0.10):
    F_roll = 0.10 × 5.0 × 9.81                      = 4.9 N
Acceleration to 0.3 m/s in 1 s (a = 0.3 m/s²):
    F_acc  = 5.0 × 0.3                              = 1.5 N
Worst-case 3° ramp / threshold:
    F_grade= 5.0 × 9.81 × sin(3°)                   = 2.6 N
                                          Total F   ≈ 9.0 N

Wheel radius r = 32.5 mm
Torque per wheel (2 driven wheels):
    τ = F × r / 2 = 9.0 × 0.0325 / 2 = 0.146 N·m    = 1.49 kgf·cm
With 2× design margin:                              = 3.0 kgf·cm required

JGB37-520 @178 RPM: rated ~3.5 kgf·cm, stall ~10 kgf·cm  ✓ 3.3× margin at stall

Top speed = 178/60 × π × 0.065 = 0.61 m/s
Cruise at 0.3 m/s = 49% duty  ✓ comfortably inside the linear region
```

### 3.10 Encoder resolution & CPU cost check (why this doesn't need a Pi)

```
JGB37-520: 11 pulses/rev at the motor shaft, gearbox ≈ 1:56
  → 11 × 56 = 616 pulses per output-shaft revolution
  → 2× quadrature decoding = 1232 counts/rev

Linear resolution = 204.2 mm / 1232 = 0.166 mm per count   ✓ far finer than needed

Interrupt load at 0.3 m/s:
  0.30 m/s ÷ 0.000166 m = 1807 counts/s per wheel
  × 2 wheels             = 3614 interrupts/s
  × ~30 cycles per ISR   = 108 k cycles/s
  ÷ 16 000 000 cycles/s  = 0.68 % CPU                      ✓ negligible

Grid A* worst case (600 cells, ~200 cycles/expansion):
  600 × 200 = 120 k cycles = 7.5 ms                        ✓ well under the 200 ms budget
Waypoint-graph Dijkstra (40 nodes):                        < 1 ms
```

These two calculations are the whole justification for an AVR brain. The workload is
proprioception and graph search on a few hundred nodes — that is a 16 MHz problem, not a
Linux problem. Perception is what would need a Pi, and perception is exactly what we replaced
with a $1.50 ultrasonic ring.

---

## 4. Pin & interface assignment

Both MCUs run at 5 V / 16 MHz, so **the UART link and all sensor signals connect directly — no
level shifters needed.** The only 3.3 V device in the whole system is the optional ESP32-CAM
(§4.5). Common ground between the two MCUs is mandatory and is the single most common cause of a
"working on the bench, garbage on the robot" serial link.

### 4.1 ATmega1284P-PU (PDIP-40) — complete pin map

| Pin | Port | Function | Connects to | Notes |
|---|---|---|---|---|
| 40 | PA0 / ADC0 | `VBAT_SENSE` | 100 kΩ / 33 kΩ divider from VBAT | 12.6 V → 3.13 V. Never exceed AREF. |
| 39 | PA1 / ADC1 | `IS_LEFT` | BTS7960 #1 IS pin | Motor current sense → stall detection |
| 38 | PA2 / ADC2 | `IS_RIGHT` | BTS7960 #2 IS pin | Same |
| 37 | PA3 / ADC3 | **SPARE (analog)** | — | Reserve for cargo-bay beam or 3rd current sense |
| 36 | PA4 | `KEYPAD_INT` | PCF8574 INT output | Active-low, key-press wake; avoids polling |
| 35 | PA5 | `DRIVER_EN` | Both BTS7960 R_EN + L_EN | LOW = coast/disable both bridges. Pulled LOW by hardware at reset. |
| 34 | PA6 | `BUZZER` | Active buzzer (via NPN if >20 mA) | |
| 33 | PA7 | `LED_GREEN` | Status LED + 220 Ω | Heartbeat / ready |
| 1 | PB0 | `ENC_L_A` (PCINT8) | Motor L hall A | Quadrature |
| 2 | PB1 | `ENC_L_B` (PCINT9) | Motor L hall B | |
| 3 | PB2 | **`SAFETY_STOP` (INT2)** | Wired-OR: bumper L, bumper R, Arduino `EMERGENCY_OUT` | Active-low, 10 kΩ pull-up. Highest-priority interrupt. |
| 4 | PB3 | `ENC_R_A` (PCINT11) | Motor R hall A | |
| 5 | PB4 | `ENC_R_B` (PCINT12) | Motor R hall B | |
| 6 | PB5 | `MOSI` | ISP header pin 4 | **Reserved for programming** |
| 7 | PB6 | `MISO` | ISP header pin 1 | **Reserved for programming** |
| 8 | PB7 | `SCK` | ISP header pin 3 | **Reserved for programming** |
| 9 | RESET | `RESET` | 10 kΩ → VCC, ISP pin 5, tactile switch | 100 nF to the ISP auto-reset line |
| 10 | VCC | +5 V | 5 V logic rail | 100 nF decoupling at the pin |
| 11 | GND | GND | Star ground point | |
| 12 | XTAL2 | Crystal | 16 MHz crystal | 22 pF to GND |
| 13 | XTAL1 | Crystal | 16 MHz crystal | 22 pF to GND |
| 14 | PD0 | `RXD0` | **Arduino D1 (TX)** | Sensor link, 38400 8N1 |
| 15 | PD1 | `TXD0` | **Arduino D0 (RX)** | Sensor link |
| 16 | PD2 | `RXD1` | USB-TTL adapter TX | Debug console |
| 17 | PD3 | `TXD1` | USB-TTL adapter RX | Debug console |
| 18 | PD4 / OC1B | `M_L_LPWM` | BTS7960 #1 LPWM | Left motor reverse |
| 19 | PD5 / OC1A | `M_L_RPWM` | BTS7960 #1 RPWM | Left motor forward |
| 20 | PD6 / OC2B | `M_R_LPWM` | BTS7960 #2 LPWM | Right motor reverse |
| 21 | PD7 / OC2A | `M_R_RPWM` | BTS7960 #2 RPWM | Right motor forward |
| 22 | PC0 | `SCL` | LCD backpack, PCF8574, MPU6050 | 4.7 kΩ pull-up to 5 V |
| 23 | PC1 | `SDA` | Same bus | 4.7 kΩ pull-up to 5 V |
| 24 | PC2 | `BUMPER_L` | Bumper microswitch L (also into INT2 OR) | For diagnostics: *which* bumper fired |
| 25 | PC3 | `BUMPER_R` | Bumper microswitch R (also into INT2 OR) | |
| 26 | PC4 | `MOTOR_RELAY` | Relay coil driver (NPN + flyback diode) | Hard-cuts the motor power rail |
| 27 | PC5 | `LED_RED` | Status LED + 220 Ω | Fault / rejected |
| 28 | PC6 | `HX711_SCK` | HX711 SCK | Bit-banged, not real SPI |
| 29 | PC7 | `HX711_DT` | HX711 DOUT | |
| 30 | AVCC | +5 V | Via 10 µH inductor + 100 nF | Clean analog supply — matters for VBAT/current readings |
| 31 | GND | GND | | |
| 32 | AREF | — | 100 nF to GND | Using AVCC as reference |

**All 32 I/O are allocated with one analog spare.** If you need more pins later, reclaim
`IS_LEFT`/`IS_RIGHT` (current sense is a nice-to-have) or move the buzzer and LEDs onto a second
PCF8574. That's the modular escape hatch.

**Timer allocation, deliberately:**

| Timer | Use | Config |
|---|---|---|
| Timer0 | `millis()` / scheduler tick | Left to the Arduino core. Do **not** repurpose. |
| Timer1 (16-bit) | Left motor PWM, OC1A + OC1B | **Fast PWM, 8-bit mode**, prescaler /8 → 7.81 kHz |
| Timer2 (8-bit) | Right motor PWM, OC2A + OC2B | Fast PWM 8-bit, prescaler /8 → 7.81 kHz |

Timer1 is forced into 8-bit mode so both motors share identical resolution and carrier frequency.
If you leave Timer1 at 16-bit, left and right wheels respond differently to the same duty value
and the robot curves — a genuinely hard bug to find. 7.81 kHz is above most audible annoyance and
comfortably inside the BTS7960's ~25 kHz limit. The ATmega1284P has **no Timer3**, so these four
outputs are all the hardware PWM you get — which is exactly enough for two motors.

### 4.2 Arduino Uno / Nano (ATmega328P) — complete pin map

All five ECHO lines are deliberately placed on **PORTB (D8–D12)** so a single `PCINT0` vector
handles every echo edge. This lets you time echoes with `micros()` inside one small ISR and
completely avoid `pulseIn()`, whose 5 × 30 ms worst-case blocking would stall the node for
150 ms per sweep.

| Pin | Function | Connects to | Notes |
|---|---|---|---|
| D0 | `RXD` | ATmega PD1 (TXD0) | Link. Occupied → no USB debug in production wiring. |
| D1 | `TXD` | ATmega PD0 (RXD0) | Link |
| D2 | `TRIG_FC` | HC-SR04 front-centre | 10 µs pulse |
| D3 | `TRIG_FL` | HC-SR04 front-left 45° | |
| D4 | `TRIG_FR` | HC-SR04 front-right 45° | |
| D5 | `TRIG_LS` | HC-SR04 left side 90° | |
| D6 | `TRIG_RS` | HC-SR04 right side 90° | |
| D7 | `EMERGENCY_OUT` | ATmega PB2 via NPN open-collector | Active-low pull-down into the wired-OR stop line |
| D8 | `ECHO_FC` (PCINT0) | HC-SR04 front-centre | |
| D9 | `ECHO_FL` (PCINT1) | HC-SR04 front-left | |
| D10 | `ECHO_FR` (PCINT2) | HC-SR04 front-right | |
| D11 | `ECHO_LS` (PCINT3) | HC-SR04 left side | |
| D12 | `ECHO_RS` (PCINT4) | HC-SR04 right side | |
| D13 | `LED_HEARTBEAT` | Onboard LED | 1 Hz blink = node alive |
| A0 | `VISION_DETECT` | ESP32-CAM GPIO (optional) | Input, pull-down |
| A1 | `VISION_BEARING0` | ESP32-CAM GPIO (optional) | 2 bits → left/centre/right |
| A2 | `VISION_BEARING1` | ESP32-CAM GPIO (optional) | |
| A3 | SPARE | — | |
| A4 / A5 | `SDA` / `SCL` | **Kept free** | Reserved for a future I2C sensor (VL53L0X ToF upgrade path) |

**Debug jumper:** put a 2-pin jumper block in the D0/D1 link. Pull it to program the Arduino over
USB and use the Serial Monitor; refit it to run on the robot. Without this you will be
disconnecting soldered wires every upload.

### 4.3 Sensor & peripheral wiring detail

**HC-SR04 (×5)** — 5 V, GND, TRIG (in), ECHO (out, 5 V). Direct connection to a 5 V ATmega328P.
Give each module its own JST-XH 4-pin connector and run all five on a shared 5 V/GND pair with a
100 nF cap at each module. Physical mounting:

```
                    front of robot
        FL 45°  ────────  FC 0°  ────────  FR 45°
           \                │                /
            \               │               /
   LS 90° ───┤        [ CARGO BAY ]        ├─── RS 90°
             │      [ 16x16x16 cm ]        │
             │                             │
             └──── WHEEL L ──── WHEEL R ───┘
                       (casters front & rear)

Mount height: 15–20 cm above floor, aimed level.
Too low → floor echoes. Too high → misses low obstacles (bags, feet).
Angle FL/FR outward 45°, sides exactly 90° to the chassis centreline.
```

**HX711 + load cell** — the load cell's 4 wires (E+, E−, A+, A−) go to the HX711; the HX711's
VCC/GND/DT/SCK go to the ATmega. Mount the load cell as a cantilever: fixed end bolted to the
lower deck, free end carrying the cargo plate, with the specified spacer gap so the beam can
actually flex. A load cell bolted flat to a plate at both ends measures nothing.

```
  [cargo plate + 16cm box]
   ══════════╤═══════════
             │ M5 bolt (free end)
        ┌────┴─────────┐
        │  LOAD CELL   │   ← must be free to deflect
        └─────────┬────┘
       M5 bolt    │ (fixed end)
   ════════════╧══════════  lower deck
```

**MPU6050** — VCC to 5 V (the GY-521 breakout has an onboard 3.3 V regulator and level-tolerant
I2C pull-ups), SDA/SCL to the shared bus, AD0 to GND (address 0x68), INT unconnected (polled).
Mount it **flat, near the robot's rotation centre**, oriented so its Z axis is vertical — only
gyro Z (yaw) is used.

**I2C bus map:**

| Address | Device |
|---|---|
| 0x20 | PCF8574 keypad expander |
| 0x27 | PCF8574 LCD backpack |
| 0x68 | MPU6050 |

Three devices, 4.7 kΩ pull-ups, 100 kHz standard mode. Keep total bus length under ~50 cm and do
not route I2C parallel to motor wires — I2C is the noise-sensitive bus in this design.

**4×4 keypad via PCF8574** — rows to P0–P3, columns to P4–P7, expander INT to ATmega PA4.
Firmware drives rows low one at a time and reads columns, exactly as a direct-connected keypad,
just through two I2C transactions.

### 4.4 Motor driver wiring (BTS7960 / IBT-2, ×2)

| BTS7960 pin | Connects to | Notes |
|---|---|---|
| B+ / B− | VBAT (post-relay, post-fuse) | 18 AWG, 1000 µF electrolytic across the terminals |
| M+ / M− | Motor terminals | 18 AWG. Add 3× 100 nF: M+→M−, M+→case, M−→case |
| RPWM | ATmega PD5 (L) / PD7 (R) | Forward PWM |
| LPWM | ATmega PD4 (L) / PD6 (R) | Reverse PWM |
| R_EN | ATmega PA5 (shared) | Both enables tied together |
| L_EN | ATmega PA5 (shared) | |
| VCC | +5 V logic rail | Logic-side supply |
| GND | Logic ground | **Also bond to power ground at the single star point only** |
| IS | ATmega PA1 (L) / PA2 (R) | Current sense, ~8.5 mV/A with the module's 1 kΩ; optional |

**Direction convention (enforce in firmware, one place only):**

| Motion | L_RPWM | L_LPWM | R_RPWM | R_LPWM |
|---|---|---|---|---|
| Forward | duty | 0 | duty | 0 |
| Reverse | 0 | duty | 0 | duty |
| Spin left (CCW) | 0 | duty | duty | 0 |
| Spin right (CW) | duty | 0 | 0 | duty |
| Brake | 0 | 0 | 0 | 0 (with `DRIVER_EN` high) |
| Coast | X | X | X | X (with `DRIVER_EN` **low**) |

**Never** drive RPWM and LPWM high simultaneously on the same module — that is a shoot-through
short across the bridge. Guard this with a single `setMotor(side, signed_duty)` function and
never write the pins anywhere else in the codebase.

### 4.5 Optional camera (ESP32-CAM) — GPIO flag interface, not serial

The camera is deliberately given the **weakest possible interface**: two or three digital lines.
This is the design decision that keeps computer vision truly optional.

| ESP32-CAM GPIO | Arduino pin | Meaning |
|---|---|---|
| GPIO12 | A0 | `VISION_DETECT` — a person/obstacle is in the forward field of view |
| GPIO13 | A1 | `VISION_BEARING0` | 
| GPIO15 | A2 | `VISION_BEARING1` — `00`=none `01`=left `10`=centre `11`=right |

The Arduino samples these three lines during its sweep and copies them into two bits of the
telemetry flags byte. The ATmega may use them **only** to bias a behaviour it would already
perform — e.g. "prefer sidestepping right because vision reports the obstacle on the left", or
"extend the wait timeout because the obstacle looks like a person who will probably move".

**It may never trigger a stop, and its absence may never change navigation.** Unplug the camera
and the flags read zero and the robot behaves exactly as the core system. That property is what
makes this a genuine option rather than a dependency.

*Level-shifting note:* the ESP32's 3.3 V output is only just above the ATmega328P's 3.0 V input
threshold at 5 V VCC. It usually reads HIGH, but it is marginal and temperature-dependent.
Configure the ESP32 pins as open-drain with 10 kΩ pull-ups to the Arduino's 5 V rail, or put a
74HCT125 buffer in between. Do not rely on the marginal direct connection.

Why not UART from the camera? A `SoftwareSerial` receive interrupt on the Arduino would corrupt
the microsecond-accurate echo timing that the entire ranging system depends on. Three GPIO lines
have zero timing impact. If you later want richer vision data, wire the ESP32-CAM to the
**ATmega's USART1** (giving up the debug console) rather than disturbing the sensor node.

---

## 5. Communication protocol: Arduino ↔ ATmega

### 5.1 Design goals

1. **Framed and checksummed** — a UART on a robot with 3 A motors *will* get bit errors. Silent
   corruption of a distance value is a collision.
2. **Fixed-size, binary** — no ASCII parsing, no `String`, no `atoi`. Parsing must be O(1) and
   allocation-free.
3. **Fail-safe on silence** — the receiver must be able to distinguish "obstacle at 5000 mm" from
   "I have heard nothing for 300 ms".
4. **Freezable in week 2** — once this contract exists, the two firmware teams can work
   independently, and the Python simulator can impersonate either side (§10.4).

### 5.2 Physical layer

| Parameter | Value | Reason |
|---|---|---|
| Interface | UART, TTL 5 V, 3 wires (TX, RX, GND) | Both MCUs are 5 V — no level shifting |
| Baud | **38400** | At 16 MHz with U2X=0, UBRR=25 gives **0.2% error**. 57600 and 115200 give 2.1% and −3.5% on this clock — enough to cause framing errors over a noisy 30 cm run. Headroom check: the telemetry stream is 360 B/s = 2880 bps, so 38400 is 13× oversized. There is no reason to take the error hit. |
| Format | 8 data, no parity, 1 stop (8N1) | Parity is redundant given the CRC |
| Flow control | None | Fixed low rate, both sides have 64 B ring buffers |
| Wiring | ATmega PD0 ← Arduino D1; ATmega PD1 → Arduino D0; **GND–GND mandatory** | Twisted pair or ribbon, routed away from motor leads |

### 5.3 Frame format

```
┌──────┬──────┬─────┬──────┬───────────────────┬───────┐
│ 0xAA │ 0x55 │ LEN │ TYPE │  PAYLOAD (LEN-1)  │ CRC8  │
└──────┴──────┴─────┴──────┴───────────────────┴───────┘
  sync1  sync2   1B    1B      0..29 bytes         1B

LEN  = number of bytes counted from TYPE up to and including the last payload byte
CRC8 = polynomial 0x07 (CRC-8/ATM), init 0x00, computed over LEN, TYPE and PAYLOAD
       (sync bytes excluded)
Byte order: little-endian (matches AVR native — no swapping code)
Max frame:  35 bytes
```

**Receiver state machine** (identical on both sides, ~40 lines of C, no allocation):

```
WAIT_SYNC1 ──0xAA──► WAIT_SYNC2 ──0x55──► READ_LEN ──► READ_TYPE
                          │  (not 0x55)                    │
                          └────────────► WAIT_SYNC1        ▼
                                                       READ_PAYLOAD
                                                            │
                                                            ▼
                                                        READ_CRC
                                                       ┌────┴────┐
                                                    match     mismatch
                                                       │         │
                                                  dispatch   drop frame,
                                                             crc_err++,
                                                             WAIT_SYNC1
```

Resynchronisation is automatic: any corrupted frame is dropped and the parser hunts for the next
`0xAA 0x55`. A stuck-high or stuck-low line simply produces no frames, which the timeout catches.

### 5.4 Message types

#### `0x01 SENSOR_TELEMETRY` — Arduino → ATmega, 20 Hz unsolicited (14-byte payload)

| Offset | Field | Type | Units / encoding |
|---|---|---|---|
| 0 | `seq` | uint8 | Increments each packet; wraps. Lets the ATmega count dropped frames. |
| 1–2 | `d_front_centre` | uint16 | mm |
| 3–4 | `d_front_left` | uint16 | mm |
| 5–6 | `d_front_right` | uint16 | mm |
| 7–8 | `d_left_side` | uint16 | mm |
| 9–10 | `d_right_side` | uint16 | mm |
| 11 | `valid_mask` | uint8 | bit0..bit4 = sensor 0..4 produced a trustworthy reading this sweep |
| 12 | `flags` | uint8 | see below |
| 13 | `sweep_age_ms` | uint8 | ms since the oldest reading in this packet (0–255). Lets the ATmega age-weight the data. |

Special distance values:

| Value | Meaning |
|---|---|
| `0x0000` | Sensor fault (no echo for N consecutive sweeps, or stuck echo line) |
| `0xFFFF` | No echo within timeout = clear beyond max range (~4000 mm) |
| 20–4000 | Valid measurement in mm |

`flags` bitfield:

| Bit | Name | Meaning |
|---|---|---|
| 0 | `EMERGENCY` | Arduino has asserted the hardware stop line (front reading < 200 mm) |
| 1 | `SENSOR_DEGRADED` | ≥1 sensor is faulted; `valid_mask` says which |
| 2 | `VISION_DETECT` | Optional camera reports something ahead |
| 3–4 | `VISION_BEARING` | 00 none, 01 left, 10 centre, 11 right |
| 5 | `SWEEP_OVERRUN` | Previous sweep did not finish in its slot (timing health) |
| 6 | `LINK_RESET` | First packet after an Arduino reset — ATmega should re-send config |
| 7 | reserved | 0 |

#### `0x02 SENSOR_FAULT` — Arduino → ATmega, on change (2-byte payload)

| Offset | Field | Meaning |
|---|---|---|
| 0 | `sensor_id` | 0=FC 1=FL 2=FR 3=LS 4=RS |
| 1 | `fault_code` | 1=no echo streak, 2=echo stuck high, 3=implausible jump, 4=recovered |

Sent once on transition, not repeatedly. The ATmega logs it and shows a degraded-mode warning on
the LCD, and refuses to enter `NAVIGATING` if a **front** sensor is faulted.

#### `0x10 SET_MODE` — ATmega → Arduino, on state change (2-byte payload)

| Offset | Field | Values |
|---|---|---|
| 0 | `mode` | 0 = IDLE (1 Hz sweep, front only — saves power and avoids pointless ranging while docked)<br>1 = NAVIGATE (5 Hz full sweep, 20 Hz reporting)<br>2 = MANOEUVRE (front trio only, 8 Hz — faster updates during a sidestep)<br>3 = SELFTEST (report raw, unfiltered) |
| 1 | `sensor_mask` | Bitmask of sensors to keep active; lets the ATmega disable a known-faulty sensor |

#### `0x11 HEARTBEAT` — ATmega → Arduino, 1 Hz (1-byte payload)

| Offset | Field | Meaning |
|---|---|---|
| 0 | `atmega_state` | Current FSM state ID — purely for the Arduino's diagnostic LED pattern |

#### `0x12 PING` / `0x13 PONG` — either direction, on demand (1-byte payload: token)

Used by the self-test at boot to measure round-trip latency and confirm both directions work
before the robot is allowed to leave `BOOT`.

### 5.5 Ultrasonic sweep schedule (why 5 Hz, and why it's safe)

HC-SR04 crosstalk is the classic failure: sensor A hears sensor B's ping and reports a phantom
close obstacle. The fix is temporal separation, but 5 sensors × 60 ms = 300 ms (3.3 Hz) is too
slow. Sensors that point **away from each other** can safely fire together:

```
Slot 0 (0 ms)    : FC                   ─ front centre alone (most safety-critical, gets a clean slot)
Slot 1 (50 ms)   : FL                   ─ front left
Slot 2 (100 ms)  : FR                   ─ front right
Slot 3 (150 ms)  : LS + RS together     ─ opposite directions, cannot hear each other
Slot 4 (200 ms)  : sweep complete → median filter → transmit packet
                   → repeat

Full sweep period : 200 ms  → 5 Hz
Telemetry rate    : 20 Hz (last-known values re-sent, with sweep_age_ms rising)
```

**Is 5 Hz enough? Stopping-distance budget at 0.3 m/s:**

```
Worst-case sensing latency (obstacle appears just after its slot)  = 200 ms → 60 mm travelled
Packet transmission + ATmega reaction (1 control period @50 Hz)    =  40 ms → 12 mm
Motor deceleration (active brake, 5 kg, ~0.6 m/s²)                 = 500 ms → 75 mm
                                                        Total stopping distance ≈ 147 mm

Thresholds chosen with margin:
  STOP threshold  = 300 mm   (2.0× stopping distance)
  SLOW threshold  = 600 mm   (scale speed linearly 100% → 30% between 600 and 300 mm)
  EMERGENCY (hw)  = 200 mm   (asserts the INT2 line directly, bypasses everything)
```

The `sweep_age_ms` field matters here: if the ATmega receives a packet whose readings are 250 ms
stale (because a sweep overran), it treats the data as less trustworthy and reduces the speed cap
accordingly. Stale range data at speed is the same hazard as no range data.

---

## 6. Complete end-to-end data flow

### 6.1 Narrative walkthrough of one delivery

```
STEP 1 — USER APPROACHES (state: IDLE)
  LCD shows: "ROBODOG READY / Press # to start"
  ATmega: HX711 tared, motors disabled (DRIVER_EN low), Arduino in mode 0 (1 Hz)

STEP 2 — DESTINATION ENTRY (IDLE → MENU_DEST_SELECT)
  User presses '#'
  ATmega: keypad expander pulls PA4 low → I2C read → key event
  LCD: "Enter room: ___"
  User types "201" then '#'
  ATmega: look up "201" in the PROGMEM room table
      ├── not found → beep, LCD "Unknown room", stay in MENU_DEST_SELECT
      └── found     → store waypoint node ID, LCD "Dest: LAB 201 / Place package"
                      → PACKAGE_LOADING

STEP 3 — PACKAGE VALIDATION (PACKAGE_LOADING)
  Dimension check is PHYSICAL: the bay is 16x16x16 cm internal.
      If it doesn't fit, it isn't loaded. No sensor, no user input, no lying.
  Weight check is MEASURED:
    ATmega reads HX711 at 10 Hz
    Requires 10 consecutive samples within +/-20 g   (stability gate — rejects a hand
                                                      still resting on the package)
    w = stable weight in grams
      ├── w < 50 g            → LCD "No package detected", wait
      ├── w > 2000 g          → RED LED + double beep
      │                         LCD "TOO HEAVY 2.4kg / Limit 2.0kg"
      │                         → PACKAGE_REJECTED → back to MENU_DEST_SELECT
      └── 50 g <= w <= 2000 g → LCD "Weight OK 1.4kg / Close lid"
                                lid microswitch closed → PIN_ISSUE

STEP 4 — PIN ISSUE (PIN_ISSUE)
  ATmega generates a 6-digit PIN (PRNG seeded at boot from floating-ADC noise + Timer1)
  LCD: "DELIVERY PIN / 4 7 2 9 1 3 / Give to receiver"
  Sender presses '#' to acknowledge → COUNTDOWN
  (The PIN is held in SRAM only for this delivery. This is the QR replacement.)

STEP 5 — COUNTDOWN (COUNTDOWN, 10 s)
  LCD counts down. Buzzer chirps at 3, 2, 1.
  ATmega: plan the global route NOW, during the countdown — planning latency is hidden
          from the user and a "no route" failure is caught before the robot moves.
    Dijkstra/A* over the waypoint graph  →  node sequence [BASE, J1, J4, C7, R201]
      ├── no route exists → LCD "No route available", return package → FAULT
      └── route found     → store leg list, DRIVER_EN high, MOTOR_RELAY on
                            send SET_MODE(1, all) to Arduino
                            → NAVIGATING

STEP 6 — NAVIGATION LOOP (NAVIGATING) — this is the core data flow, 50 Hz
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │ (a) ARDUINO, continuously at 5 Hz sweep / 20 Hz report:                     │
  │       fire FC → FL → FR → (LS+RS) → median filter → mm                      │
  │       if any front reading < 200 mm: pull D7 LOW  ──hardware──►  ATmega INT2 │
  │       build SENSOR_TELEMETRY, CRC8, send over UART @38400                    │
  │                                       │                                      │
  │ (b) ATmega UART0 ISR → ring buffer → frame parser → validated telemetry      │
  │       if no valid packet for 250 ms → SAFE_STOP                              │
  │                                       │                                      │
  │ (c) ATmega ODOMETRY, 100 Hz (independent of the sensor link):                │
  │       PCINT ISRs accumulate encoder counts (0.166 mm/count)                  │
  │       dL, dR → ds = (dL+dR)/2 ;  dtheta_enc = (dR-dL)/track_width            │
  │       MPU6050 gyro Z integrated → dtheta_gyro                                │
  │       fused: theta += 0.02*dtheta_enc + 0.98*dtheta_gyro   (gyro dominates   │
  │              heading; encoders dominate distance)                            │
  │       x += ds*cos(theta) ;  y += ds*sin(theta)                               │
  │                                       │                                      │
  │ (d) ATmega LOCAL ARBITRATION, 50 Hz — combines (b) and (c):                  │
  │       speed_cap = 1.0                                                        │
  │       for each front sensor:                                                 │
  │           if d < 300 mm  → speed_cap = 0        (STOP)                       │
  │           elif d < 600 mm→ speed_cap = min(cap, 0.3 + 0.7*(d-300)/300)       │
  │       if sweep_age_ms > 150 → speed_cap *= 0.5   (stale data penalty)        │
  │       lateral_correction = PID(d_left_side - d_right_side)   ← corridor       │
  │                                                               centring       │
  │       heading_error = leg_bearing - theta  (+ lateral_correction)            │
  │                                       │                                      │
  │ (e) ATmega MOTOR OUTPUT, 100 Hz:                                             │
  │       v_target = cruise_speed * speed_cap                                    │
  │       w_target = heading_PID(heading_error)                                  │
  │       v_L = v_target - w_target*track/2  ;  v_R = v_target + w_target*track/2│
  │       duty_L = velocityPID_L(v_L, measured_from_encoders)                    │
  │       duty_R = velocityPID_R(v_R, measured_from_encoders)                    │
  │       setMotor(LEFT, duty_L) → PD5/PD4 PWM → BTS7960 #1 → motor L → wheel L  │
  │       setMotor(RIGHT, duty_R) → PD7/PD6 PWM → BTS7960 #2 → motor R → wheel R │
  │                                       │                                      │
  │ (f) ATmega WAYPOINT PROGRESS, 20 Hz:                                         │
  │       distance travelled along leg vs expected leg length                    │
  │       side-sonar opening detected within +/-300 mm of an expected junction    │
  │           → snap odometry to that junction's map coordinate (drift reset)    │
  │       leg complete → gyro-closed-loop turn to next leg bearing → next leg    │
  │       final leg complete → ARRIVED                                           │
  │                                       │                                      │
  │ (g) If speed_cap has been 0 for >2 s → OBSTACLE_WAIT (see §7.4 ladder)       │
  └─────────────────────────────────────────────────────────────────────────────┘

STEP 7 — ARRIVAL (ARRIVED_WAIT_RECEIVER)
  ATmega: motors stopped, DRIVER_EN low, SET_MODE(0) to Arduino
  Buzzer: 3 chirps. LCD: "DELIVERY FOR / LAB 201 / Enter PIN: ______"
  Timeout 120 s with no valid PIN → RETURNING (undelivered, logged)

STEP 8 — PIN VERIFICATION (VERIFY_PIN)
  Receiver types 6 digits + '#'
    ├── mismatch → beep, attempts++, LCD "Wrong PIN (2 left)"
    │              3 failures → 60 s lockout, then back to ARRIVED_WAIT_RECEIVER
    └── match    → LCD "Verified. Open lid / and take package" → HANDOVER

STEP 9 — HANDOVER CONFIRMATION (HANDOVER)
  ATmega watches the HX711: weight must drop below (w_loaded - 50 g) and stay stable 2 s
    → the load cell doubles as the package-presence sensor. No extra part needed.
  Lid closed again → LCD "Thank you" → RETURNING

STEP 10 — RETURN (RETURNING)
  Same navigation loop as STEP 6, target = BASE node.
  Arrival at base → DOCKED → IDLE, HX711 re-tared, PIN wiped from SRAM.
```

### 6.2 Data-flow diagram (one control cycle)

```
  USER ──keypress──► PCF8574 ──I2C──► ┐
  LOAD CELL ──mV──► HX711 ──2-wire──► ├──► ┌──────────────────────┐
  MPU6050 ──yaw rate──I2C───────────► ┤    │  ATmega1284P         │
  ENCODERS ──quadrature──PCINT──────► ┤    │                      │
                                      │    │  FSM  +  PLANNER     │
  5x HC-SR04 ──echo──► ARDUINO ──UART►┤    │    +  ODOMETRY       │──PWM──► BTS7960 ──► MOTORS
  BUMPERS ──────────────wired-OR─────►┘    │    +  PID  +  SAFETY │           x2         x2
  ESP32-CAM ──GPIO──► ARDUINO ─────────►   └──────────┬───────────┘             │
                                                      │                          ▼
                                              I2C ────▼───► LCD 20x4          WHEELS
                                                            BUZZER, LEDs         │
                                                                                 ▼
                                                                            ODOMETRY
                                                                          (closes the loop)
```

Note the loop closure: wheel motion is measured by the encoders and fed back into odometry, and
corridor geometry is measured by the side sonars and fed back into position. **There is no open-loop
motion anywhere** — no `forward(2 seconds)` style commands. That single rule is the difference
between a robot that reaches room 201 and a robot that reaches "somewhere near room 201".

---

## 7. Navigation & rerouting logic for a predefined indoor map

### 7.1 Use a topological waypoint graph on the robot, not the 20×30 grid

This is the most important navigation decision in the document, and it is a change from the
current Python design.

A 30 cm grid cell demands that the robot know its position to better than ±15 cm to know which
cell it is in. A wheel-odometry robot with no LiDAR accumulates roughly **2–5% of distance
travelled** as position error. Over a 20 m corridor that is 40–100 cm — three cells of error. The
grid planner would be producing precise cell-by-cell plans for a robot that fundamentally does
not know which cell it occupies. The plan would be fiction.

A **topological graph** matches what the robot can actually sense:

```
        R101         R102              R201        LAB1
          │            │                 │           │
   BASE──J1───────────J2────────────────J3──────────J4
         corridor A         corridor B      corridor C
        (5.2 m, 0°)        (8.4 m, 0°)     (6.0 m, 0°)
          │
          J5 ── (corridor D, 4.0 m, 90°) ── R103
```

| Element | Content | Storage |
|---|---|---|
| **Node** | `id`, `x_cm`, `y_cm`, `type` (BASE / JUNCTION / DOOR), `label[8]` | 8 B × ~40 nodes = 320 B in PROGMEM |
| **Edge** | `node_a`, `node_b`, `length_cm`, `bearing_deg`, `width_cm`, `base_cost` | 8 B × ~50 edges = 400 B in PROGMEM |
| **Room table** | `room_number[5]` → `node_id` | 6 B × ~20 rooms = 120 B in PROGMEM |
| **Runtime blocked-edge overlay** | 1 bit + timestamp per edge | 50 bits + 50 × 2 B = ~106 B SRAM |

Total map footprint: **under 1 KB of flash, ~100 B of SRAM.** Dijkstra over 40 nodes completes in
well under 1 ms. Compare with the ~1.4 KB SRAM and 7.5 ms for the grid version — and the graph is
*more* accurate in practice because every node is a place the robot can physically recognise.

**The grid map stays in the Python simulator.** Write an offline converter
(`tools/grid_to_graph.py`) that reads your existing `gridmap.py` / `campuslayout.py` output and
emits a C header of nodes and edges. Then the simulator and the robot are guaranteed to plan over
the same topology, and your existing A\* work is not wasted — it becomes the ground-truth
reference the graph is validated against.

**Each edge must be traversable by a single behaviour:** "drive straight down this corridor at
this bearing, centred between the walls, for this distance". If an edge cannot be described that
way, split it at a junction node. This is the rule that keeps the local controller simple.

### 7.2 Two-layer architecture

| Layer | Rate | Input | Output | Knows the map? |
|---|---|---|---|---|
| **Global planner** | on demand | Graph + blocked-edge overlay + goal | Ordered node list | Yes |
| **Leg controller** | 50 Hz | Current leg (bearing, length), odometry, sonar | v, ω setpoints | No — only the current leg |

The leg controller is deliberately map-blind. It executes one straight corridor run and reports
`LEG_DONE`, `LEG_BLOCKED` or `LEG_LOST`. Everything strategic lives in the planner. This keeps the
50 Hz path free of graph search and makes both halves independently testable.

### 7.3 Localisation & drift correction (the part that actually determines success)

Dead reckoning alone is not enough. Three correction mechanisms, all using hardware you already
have:

**(a) Corridor centring — continuous lateral correction.**
```
error = d_left_side - d_right_side          (mm, both valid)
lateral_PID → small ω bias
Result: the robot converges to the corridor centreline and holds it.
Guard: only active when BOTH side readings are valid and < 2000 mm.
       In an open area, disable and fall back to gyro heading hold.
```
This eliminates lateral drift entirely, which is the error component that actually causes
collisions. It costs nothing — the sensors are already there for obstacle detection.

**(b) Gyro heading hold — eliminates angular drift.**
```
Straight legs: hold the leg's map bearing using fused yaw, not "equal wheel speeds".
Turns:  closed loop on integrated gyro Z to the target bearing, ±2° tolerance,
        then verify: post-turn side-sonar readings must be consistent with the new
        corridor's known width (±100 mm). If not → LEG_LOST → recovery (§7.5).
```

**(c) Junction snapping — eliminates longitudinal drift.**
```
A junction is a step change in a side reading:
    d_left_side jumps from ~800 mm (wall) to >2000 mm (opening) and holds for >200 mm
    of travel → an opening on the left.

If an opening is detected while odometry says we are within ±300 mm of an expected
junction node:
    → CONFIRMED. Snap x,y to that node's map coordinates. Longitudinal error → 0.

If odometry says we passed the expected junction by >500 mm with no opening detected:
    → LEG_LOST. We are not where we think we are. Stop, run recovery.
```

This is a cheap, robust, LiDAR-free localisation scheme: **error accumulates only within one
corridor leg and is zeroed at every junction.** Keep legs under ~8 m and worst-case in-leg error
stays under ~40 cm longitudinally, with lateral error held near zero by (a). That is well inside
the tolerance for stopping at a door.

Design consequence: **when surveying the building, place a node at every doorway and corridor
opening**, even ones you never route through. Every opening is a free position fix.

### 7.4 Obstacle response ladder (rerouting policy)

Escalate in strict order. Never skip a rung. This ordering is chosen because indoor obstacles are
overwhelmingly **people**, and people move.

```
RUNG 1 — SLOW  (front reading 300–600 mm)
    Scale speed 100% → 30% proportionally. Chirp the buzzer once per second.
    Cost: nothing. Resolves most encounters, because a human who hears a slowing robot
    steps aside. Stay in NAVIGATING.

RUNG 2 — STOP AND WAIT  (front reading < 300 mm)
    Full stop, hold position, LCD "Waiting — please excuse me", buzzer every 2 s.
    Wait up to 8 s (12 s if VISION_DETECT suggests a person).
    → path clears  : resume, back to RUNG 1
    → still blocked: RUNG 3
    Cost: nothing. This rung alone handles the large majority of real corridor blockages.
    Skipping it is the single most common design mistake — an immediate reroute around a
    person who was about to move wastes 30 s and often fails.

RUNG 3 — SIDESTEP  (bounded local manoeuvre, corridor only)
    Preconditions, ALL required:
        - current edge width_cm >= robot_width + 500 mm   (from the map, not guessed)
        - the chosen side reading > 700 mm (real clearance exists)
        - fewer than 2 sidesteps already used on this leg
    Manoeuvre: lateral shift of 300 mm via a short arc-straight-arc, gyro-controlled,
        then resume the leg bearing. Prefer the side OPPOSITE to VISION_BEARING if
        available, otherwise the side with more clearance.
    Odometry impact is logged; the next junction snap corrects it.
    → success: resume leg
    → still blocked, or preconditions unmet: RUNG 4

RUNG 4 — REPLAN  (global)
    Mark the current edge BLOCKED with a timestamp (60 s decay — this mirrors the
    existing OBSTACLE_DECAY_SECONDS idea in settings.py, applied to edges).
    Re-run Dijkstra from the last CONFIRMED node to the goal.
    → alternate route found: reverse to the last junction, take the new route
    → no route found       : RUNG 5

RUNG 5 — ABORT
    If carrying a package : plan a route back to BASE, LCD "Delivery failed — returning",
                            log the reason. If BASE is also unreachable → RUNG 6.
    If already returning  : RUNG 6.

RUNG 6 — PARK AND ALERT
    Stop in place, hug the nearest wall if possible, DRIVER_EN low, MOTOR_RELAY off.
    LCD "Blocked — assistance needed", buzzer pattern every 10 s, red LED.
    Requires human intervention (keypad '*' + operator code) to clear.
    Never resume autonomously from RUNG 6: if the robot could not solve it in 5 rungs,
    it does not understand its situation and must not keep moving.
```

**Edge-cost inflation instead of hard blocking (optional refinement):** rather than marking an
edge fully blocked, multiply its cost by 4 on the first failure and by 16 on the second. The
planner then naturally prefers alternatives but will still use the edge if it is the only way
through. This avoids the "no route found" cliff in a building with few redundant corridors.

### 7.5 `LEG_LOST` recovery

If junction snapping fails (expected opening never appeared, or a post-turn sanity check failed),
the robot's belief about its position is wrong and continuing is dangerous.

```
1. Stop immediately. Do not guess.
2. Rotate slowly in place, sampling all 5 sonars every 15° for a full 360°.
   This gives a crude 24-point range signature of the current location.
3. Compare the signature with the expected signature of each nearby node
   (corridor width, number and bearing of openings — all derivable from the map).
4. Best match within tolerance → re-localise to that node, replan, resume.
   No confident match → RUNG 6 (park and alert).
```

This is not SLAM. It is a 24-point template match against ~5 candidate nodes, a few hundred bytes
of comparison. It runs on an AVR in milliseconds and recovers the majority of lost-position events
without human help.

### 7.6 Rules that keep this buildable

| Rule | Reason |
|---|---|
| The robot stops at a corridor node **outside** the room, never drives in | Room interiors are unmapped, cluttered and full of chair legs. Eliminates the hardest 20% of the problem for zero loss of function. |
| Legs are straight corridor runs only; every turn happens at a node, in place | One behaviour to tune instead of arbitrary curve following. |
| Maximum leg length 8 m | Bounds in-leg odometry drift to ~40 cm. |
| Cruise speed 0.3 m/s, manoeuvre speed 0.15 m/s | Keeps the 5 Hz sonar sweep sufficient (§5.5) and makes the robot socially acceptable in a corridor. |
| Single floor, no stairs, no elevator, no ramps > 3° | Removes the largest mechanical and safety risk class entirely. |
| The map is surveyed with a tape measure and committed to version control | The map is source code. Measure corridor lengths and widths once, carefully, and treat corrections as commits. Guessed coordinates are the #1 cause of "the robot drives into a wall". |

---

## 8. Robot state machine

### 8.1 Diagram

```
                          ┌──────────┐
             power on ───►│   BOOT   │
                          └────┬─────┘
                               │ init peripherals
                          ┌────▼─────┐
                          │SELF_TEST │  I2C scan, HX711 alive, PING/PONG to Arduino,
                          └────┬──┬──┘  all 5 sonars report, encoders respond to a
                    pass       │  │ fail 5 cm nudge, VBAT > 10.5 V
              ┌────────────────┘  └────────────────┐
              │                                    ▼
         ┌────▼─────┐  '#'          ┌──────────────────────┐
    ┌───►│   IDLE   ├──────────────►│  MENU_DEST_SELECT    │
    │    └────┬─────┘               └──┬────────────────┬──┘
    │         │ VBAT<10.0V              │ room valid     │ '*' cancel
    │         │                         ▼                └────────┐
    │    ┌────▼──────────┐    ┌───────────────────┐               │
    │    │LOW_BATT_RETURN│    │ PACKAGE_LOADING   │               │
    │    └───────────────┘    └──┬─────────────┬──┘               │
    │                            │ w<=2000g    │ w>2000g          │
    │                            │ & stable    ▼                  │
    │                            │      ┌──────────────────┐      │
    │                            │      │PACKAGE_REJECTED  ├──────┤
    │                            │      └──────────────────┘      │
    │                            ▼                                │
    │                     ┌──────────────┐                        │
    │                     │  PIN_ISSUE   │  6-digit PIN on LCD    │
    │                     └──────┬───────┘                        │
    │                            │ sender ack '#'                  │
    │                     ┌──────▼───────┐   '*' abort             │
    │                     │  COUNTDOWN   ├────────────────────────►┤
    │                     │  10 s + plan │                         │
    │                     └──────┬───────┘  no route               │
    │                            │          ────────────────► FAULT│
    │        ┌───────────────────▼──────────────────────┐          │
    │        │             NAVIGATING                   │          │
    │        │  leg controller + local avoidance        │◄────┐    │
    │        └──┬────────┬─────────┬──────────┬─────────┘     │    │
    │           │        │         │          │               │    │
    │  arrived  │  d<300 │ LEG_    │ RUNG 4   │ link timeout  │    │
    │           │  >2 s  │ LOST    │          │ / bumper /    │    │
    │           │        │         │          │ INT2          │    │
    │           │   ┌────▼──────┐  │    ┌─────▼──────┐  ┌─────▼──┐ │
    │           │   │OBSTACLE_  │  │    │ REPLANNING │  │SAFE_   │ │
    │           │   │WAIT (8s)  ├──┼───►│            │  │STOP    │ │
    │           │   └────┬──────┘  │    └─────┬──────┘  └──┬───┬─┘ │
    │           │  clear │         │  route   │            │   │   │
    │           │        └─────────┼──────────┘            │   │   │
    │           │                  │                clear  │   │   │
    │           │            ┌─────▼──────┐         (5 s)   │   │   │
    │           │            │  RECOVERY  │─────────────────┼───┘   │
    │           │            │ 360° scan  │                 │       │
    │           │            └─────┬──────┘         persist │       │
    │           │        no match  │                        ▼       │
    │           │                  ▼                  ┌──────────┐  │
    │           │            ┌──────────┐             │  FAULT   │  │
    │           │            │PARK_ALERT│◄────────────┤ / E_STOP │  │
    │           │            └──────────┘  no route   └────┬─────┘  │
    │           │                                          │ '*' +  │
    │  ┌────────▼──────────────┐                           │ op code│
    │  │ ARRIVED_WAIT_RECEIVER │                           └────────┤
    │  └────────┬──────────────┘  120 s timeout ──────────┐         │
    │           │ 6 digits + '#'                          │         │
    │     ┌─────▼──────┐  3x wrong → 60 s lockout         │         │
    │     │ VERIFY_PIN ├──────────────┐                    │         │
    │     └─────┬──────┘              │                    │         │
    │           │ match               └───► back to ARRIVED_WAIT     │
    │     ┌─────▼──────┐                                   │         │
    │     │  HANDOVER  │  weight drop > 50 g, stable 2 s   │         │
    │     └─────┬──────┘  + lid closed                     │         │
    │           │                                          │         │
    │     ┌─────▼──────┐◄────────────────────────────────── ┘        │
    │     │ RETURNING  │  (same nav loop, target = BASE)             │
    │     └─────┬──────┘                                            │
    │     ┌─────▼──────┐                                            │
    └─────┤   DOCKED   │  re-tare HX711, wipe PIN, motors off       │
          └────────────┘                                            │
                                                                     │
          (cancel paths return to IDLE) ◄────────────────────────────┘
```

### 8.2 State table

| State | Entry action | Exit condition(s) | Motors | Arduino mode | Timeout |
|---|---|---|---|---|---|
| `BOOT` | Init clocks, ports, timers, UARTs, WDT | Init complete | Disabled | — | — |
| `SELF_TEST` | Run all checks (§8.4) | All pass → `IDLE`; any fail → `FAULT` | Disabled | 3 (selftest) | 5 s |
| `IDLE` | Tare HX711, LCD ready screen, relay off | `#` → `MENU_DEST_SELECT`; VBAT<10.0 → `LOW_BATT_RETURN` | Disabled | 0 (1 Hz) | — |
| `MENU_DEST_SELECT` | LCD room prompt | Valid room → `PACKAGE_LOADING`; `*` → `IDLE` | Disabled | 0 | 60 s → `IDLE` |
| `PACKAGE_LOADING` | LCD weight live display | Stable & ≤2000 g & lid closed → `PIN_ISSUE`; >2000 g → `PACKAGE_REJECTED` | Disabled | 0 | 120 s → `IDLE` |
| `PACKAGE_REJECTED` | Red LED, double beep, show actual vs limit | 5 s → `MENU_DEST_SELECT` | Disabled | 0 | 5 s |
| `PIN_ISSUE` | Generate & display 6-digit PIN | `#` → `COUNTDOWN`; `*` → `IDLE` | Disabled | 0 | 60 s → `IDLE` |
| `COUNTDOWN` | Plan route; 10 s countdown; chirp 3-2-1 | 0 → `NAVIGATING`; no route → `FAULT`; `*` → `IDLE` | Disabled → enabled at 0 | 0 → 1 | 10 s |
| `NAVIGATING` | Relay on, `DRIVER_EN` high, `SET_MODE(1)` | Goal → `ARRIVED_*`/`DOCKED`; blocked>2 s → `OBSTACLE_WAIT`; `LEG_LOST` → `RECOVERY`; safety → `SAFE_STOP` | **Active** | 1 (5 Hz) | 300 s/leg → `FAULT` |
| `OBSTACLE_WAIT` | Stop, LCD apology, periodic chirp | Clear → `NAVIGATING`; timeout → sidestep or `REPLANNING` | Held (0 duty, brake) | 2 (8 Hz) | 8 s (12 s if vision) |
| `REPLANNING` | Mark edge blocked, re-run Dijkstra | Route → `NAVIGATING`; none → `PARK_ALERT` / return | Stopped | 1 | 1 s |
| `RECOVERY` | 360° scan, template match | Match → `NAVIGATING`; none → `PARK_ALERT` | Slow spin 0.15 m/s | 1 | 30 s |
| `ARRIVED_WAIT_RECEIVER` | Chirp ×3, LCD PIN prompt, relay off | 6 digits+`#` → `VERIFY_PIN` | Disabled | 0 | 120 s → `RETURNING` |
| `VERIFY_PIN` | Compare | Match → `HANDOVER`; 3 fails → 60 s lockout → `ARRIVED_WAIT_RECEIVER` | Disabled | 0 | — |
| `HANDOVER` | LCD "take package" | Weight drop >50 g stable 2 s + lid closed → `RETURNING` | Disabled | 0 | 120 s → `RETURNING` |
| `RETURNING` | Plan route to BASE, relay on | At BASE → `DOCKED`; same failure paths as `NAVIGATING` | **Active** | 1 | 300 s/leg |
| `DOCKED` | Re-tare, wipe PIN, relay off, log delivery | Immediate → `IDLE` | Disabled | 0 | — |
| `LOW_BATT_RETURN` | LCD warning | At BASE → `FAULT` (await charge) | Active (0.2 m/s) | 1 | — |
| `SAFE_STOP` | **Brake, `DRIVER_EN` low within 1 ms**, LCD reason | Cause clear 5 s → `NAVIGATING`; persists 15 s → `FAULT` | Braked then disabled | 1 | 15 s |
| `PARK_ALERT` | Stop, relay off, alert pattern | `*`+operator code → `IDLE` | Disabled | 0 | — |
| `FAULT` / `E_STOP` | Relay off, `DRIVER_EN` low, red LED, LCD fault code | `*`+operator code → `SELF_TEST` | Disabled | 0 | — |

### 8.3 Mapping to the existing Python FSM

Your `core/state_machine.py` has 11 states. The embedded FSM has 20 — the extra states are all
things a simulation can leave implicit but hardware cannot.

| Existing Python state | Embedded equivalent | Note |
|---|---|---|
| `IDLE` | `IDLE` | plus `BOOT`, `SELF_TEST`, `DOCKED` |
| `VERIFYING_SENDER_QR` | *(removed)* | No camera. The sender is whoever is standing there. |
| `FORM_INPUT` | `MENU_DEST_SELECT` + `PACKAGE_LOADING` | Split: destination entry vs measured weight |
| `PACKAGE_ACCEPTED` | `PIN_ISSUE` | Acceptance now also issues the PIN |
| `COUNTDOWN` | `COUNTDOWN` | Now also the planning window |
| `NAVIGATING` | `NAVIGATING` + `OBSTACLE_WAIT` + `REPLANNING` + `RECOVERY` | Sub-behaviours become explicit states so the LCD can report *why* the robot is stopped |
| `AT_DESTINATION` | `ARRIVED_WAIT_RECEIVER` | |
| `VERIFYING_RECEIVER_QR` | `VERIFY_PIN` | Keypad PIN replaces QR |
| `DELIVERING` | `HANDOVER` | Now confirmed by the load cell, not by a click |
| `RETURNING` | `RETURNING` + `LOW_BATT_RETURN` | |
| `ERROR` | `SAFE_STOP` + `FAULT` + `PARK_ALERT` | Split by recoverability — the critical distinction |
| — | `PACKAGE_REJECTED` | New: needs a dwell state to actually show the user the reason |

**Recommendation:** update the Python FSM to this 20-state model *before* writing firmware, and
port the transition table verbatim. Then `ztest/test_statemachine.py` becomes the specification
test for the firmware FSM, and any transition bug is caught in Python in seconds instead of on a
robot in an hour.

### 8.4 Self-test checklist (gate on every boot — do not skip)

| Check | Method | Fail action |
|---|---|---|
| I2C bus | Scan for 0x20, 0x27, 0x68 | `FAULT`, code E01 |
| LCD | Write a known pattern | `FAULT` E02 (blind robot = no fault reporting) |
| Keypad | Expander readable, no key stuck low | `FAULT` E03 |
| HX711 | DOUT goes low within 200 ms, reading within plausible range | `FAULT` E04 |
| Load cell zero | Tare within ±200 g of stored zero | Warn W01, allow with re-tare |
| MPU6050 | WHO_AM_I = 0x68, gyro Z bias < 5 °/s at rest | Warn W02, degrade to encoder-only turns |
| Arduino link | PING → PONG within 100 ms | `FAULT` E05 |
| All 5 sonars | Each reports a plausible value in `SELFTEST` mode | Front sensor fail → `FAULT` E06; side fail → Warn W03 (no corridor centring) |
| Encoders | 5 cm nudge each wheel, both count in the right direction | `FAULT` E07 (this catches swapped A/B wiring, a classic) |
| Battery | VBAT ≥ 10.5 V | `FAULT` E08 |
| Bumpers | Both read released | `FAULT` E09 |
| E-stop | Motor relay responds | `FAULT` E10 |

The encoder nudge test is worth the awkwardness. A reversed encoder channel makes the velocity PID
run away in the wrong direction, and it is nearly impossible to diagnose from behaviour alone.

### 8.5 Firmware discipline for the AVR (non-negotiable rules)

| Rule | Why |
|---|---|
| Cooperative scheduler on `millis()` slots, **no RTOS** | 20 states and 6 periodic tasks do not justify an RTOS's RAM cost on a 16 KB chip |
| **No `delay()` anywhere** except in `SELF_TEST` | A single `delay(100)` in a control loop is a 100 ms blind spot at 0.3 m/s = 3 cm |
| **No Arduino `String` class, no `malloc`/`new`** | Heap fragmentation on a 16 KB AVR is a slow, random, undebuggable death. Fixed `char[]` buffers only. |
| Constant tables and all literal strings in `PROGMEM` | Keeps SRAM for the planner and stack |
| Watchdog enabled at 1 s, reason latched in a no-init RAM variable | Turns a mystery reset into a logged fault code |
| ISRs do counting and buffering only — never I2C, never floating point, never LCD | An ISR that takes 200 µs will corrupt echo timing and encoder counts |
| Every motor write goes through one `setMotor(side, signed_duty)` function | Single place to enforce the no-shoot-through and enable-state invariants |
| Log state transitions to USART1 with a timestamp | This log is your entire debugging capability on a moving robot |
| Track free SRAM (stack-painting) and print it in the debug log | Gives early warning before a stack overflow reboots the robot mid-corridor |

---

## 9. Power architecture

### 9.1 Rail topology

```
  ┌─────────────────────────┐
  │ 3S Li-ion 18650 pack    │  9.0 V (empty) – 12.6 V (full), nominal 11.1 V
  │ 11.1 V  6.8 Ah  + BMS   │  BMS: over-discharge, over-current, short protection
  └────────────┬────────────┘
               │ XT60
        ┌──────▼──────┐
        │ 10 A blade  │  ← the only thing between a shorted motor lead and a fire
        │    fuse     │
        └──────┬──────┘
               │
        ┌──────▼──────┐
        │ MAIN SWITCH │  (rocker, whole robot)
        └──────┬──────┘
               │
     ┌─────────┴──────────────────────────────────┐
     │                                            │
     │  ═══ MOTOR BRANCH ═══                      │  ═══ LOGIC BRANCH ═══
     │                                            │
┌────▼──────────────┐                    ┌────────▼──────────┐
│ E-STOP (NC latch) │                    │ LM2596 buck 5V 3A │
└────┬──────────────┘                    └────────┬──────────┘
     │                                            │  5 V LOGIC RAIL
┌────▼──────────────┐                             ├──► ATmega1284P      15 mA
│ RELAY 12V 30A     │◄── ATmega PC4               ├──► Arduino (5V pin,  45 mA
│ (motor power cut) │    (MOTOR_RELAY)            │    bypassing its regulator)
└────┬──────────────┘                             ├──► 5x HC-SR04        75 mA pk
     │                                            ├──► LCD + backlight   30 mA
     │  VBAT_MOTOR (post-relay)                   ├──► HX711 + cell      10 mA
     ├──► 1000 µF ──► BTS7960 #1 ──► MOTOR L      ├──► MPU6050            4 mA
     └──► 1000 µF ──► BTS7960 #2 ──► MOTOR R      ├──► PCF8574 + keypad   5 mA
                                                  ├──► buzzer + 3 LEDs   25 mA
     VBAT (pre-relay) ──► 100k/33k ──► ADC0       └──► relay coil        70 mA
                          (VBAT_SENSE)                 (optional ESP32   200 mA avg)

  GROUNDING: single star point at the battery negative terminal.
  Motor ground and logic ground meet ONLY there. Never daisy-chain.
```

### 9.2 Why each power decision is what it is

| Decision | Reason |
|---|---|
| **Switching buck (LM2596), not a linear regulator** | Dropping 11.1 V → 5 V at 280 mA linearly dissipates (11.1−5)×0.28 = **1.7 W**. The Arduino's onboard AMS1117 in a TO-252 package is rated for well under that without a heatsink; it will thermal-shutdown mid-delivery. A buck at ~85% efficiency dissipates 0.25 W. **Feed the Arduino through its 5 V pin from the buck, not through VIN or the barrel jack.** |
| **E-stop and relay in the motor branch only** | If the E-stop killed everything, the robot would go dark and could not tell you why it stopped. Logic stays alive, the LCD shows the fault, the debug log keeps recording. |
| **Relay in addition to the E-stop** | Gives the ATmega a hardware kill it can assert itself (over-current, `PARK_ALERT`, watchdog reset). A MOSFET/`DRIVER_EN` low is a *soft* disable; the relay is galvanic. |
| **1000 µF at each driver's input** | A DC motor's commutation and reversal current spikes will drag VBAT down tens of millivolts per amp of wiring resistance. The bulk cap supplies the transient locally so the dip never reaches the buck input. |
| **100 nF across each motor's terminals and to its case** | Brush arcing radiates broadband EMI. This is the standard, cheap, effective fix for the classic "MCU resets when the motor starts" bug. Do it during the build, not after you've wasted a week. |
| **Star ground, separate motor and logic returns** | Motor current sharing a return path with logic ground creates a ground bounce that appears as noise on the ADC and can corrupt I2C. This is a wiring decision that cannot be fixed in firmware. |
| **Li-ion + BMS over bare LiPo** | Comparable energy density, dramatically better failure behaviour, and safe to leave in a lab drawer between demo days. A 3S LiPo with no protection board on a student robot that will be short-circuited at least once is a real fire risk. |
| **Two 5 V buck modules purchased, one installed** | These modules are the second most common thing to fail after motor drivers. A spare in the toolbox turns a dead demo into a five-minute swap. |

### 9.3 Current & runtime budget

| Condition | Logic | Motors | Total @11.1 V | Power | Runtime (6.8 Ah, 80% usable) |
|---|---|---|---|---|---|
| Idle / docked | 210 mA | 0 | 0.21 A | 2.3 W | ~26 h |
| Cruising 0.3 m/s, flat | 280 mA | 2 × 420 mA | 1.12 A | 12.4 W | **~4.9 h** |
| Cruising with a 2 kg load | 280 mA | 2 × 600 mA | 1.48 A | 16.4 W | **~3.7 h** |
| Turning in place | 280 mA | 2 × 750 mA | 1.78 A | 19.8 W | ~3.1 h |
| With ESP32-CAM active | +200 mA | — | +0.20 A | +2.2 W | −15% |
| Stall (transient, fault) | 280 mA | 2 × 2800 mA | 5.9 A | 65 W | fuse survives, BMS may trip |

**Realistic mixed-duty estimate: 2.5–3 h of continuous delivery operation**, or roughly 30–40
short deliveries. Comfortably more than a demo day needs, with margin for the battery ageing.

Peak draw is 5.9 A against a 10 A fuse — correct sizing (fuse above worst legitimate transient,
below the wiring's capability). 18 AWG silicone handles ~7–10 A continuously in this length.

### 9.4 Voltage thresholds and monitoring

`VBAT_SENSE` = VBAT × 33/(100+33) = VBAT × 0.248. At 12.6 V that is 3.13 V — safely inside the
5 V ADC range with good resolution (~13 mV per LSB of battery voltage).

| VBAT | Per cell | Action |
|---|---|---|
| 12.6 V | 4.20 V | Fully charged |
| 11.1 V | 3.70 V | Nominal — normal operation |
| 10.5 V | 3.50 V | **Warn.** LCD low-battery icon, refuse to start a *new* delivery. Finish the current one. |
| 10.0 V | 3.33 V | **Force `LOW_BATT_RETURN`.** Abort delivery, drive to base at 0.2 m/s. |
| 9.6 V | 3.20 V | **Shutdown.** Motor relay off, LCD "Charge me", logic sleeps. |
| < 9.0 V | 3.00 V | BMS cuts off. Below this you damage cells. |

Read VBAT with a **16-sample rolling average**, and only ever act on a threshold using a reading
taken while motors are at low duty. Instantaneous voltage during acceleration sags 0.5–1.0 V under
load; acting on that sag will make the robot abort deliveries the moment it starts moving. This is
a real bug that appears in almost every first-attempt battery monitor.

### 9.5 Brownout protection

Set the ATmega1284P's **BOD fuse to 4.3 V** and enable the watchdog. If the 5 V rail dips below
4.3 V (motor inrush overwhelming the buck), the MCU resets cleanly and deterministically rather
than executing corrupted instructions with the motors still enabled. On reset, `DRIVER_EN` and
`MOTOR_RELAY` are inputs/low by hardware default, so the motors are dead before firmware runs a
single line. **Design the reset state to be the safe state** — verify this with a multimeter before
the first powered drive test.

---

## 10. Recommended simulators

No Raspberry Pi means no Gazebo/ROS-class simulation is needed — and adding ROS would drag
Pi-class compute back into the design through the back door. Use a **layered** approach: each tool
validates the layer it is actually good at.

### 10.1 Layer 1 — Algorithms & FSM: your existing Python simulator ⭐ primary

**Keep and extend what's already in this repo.** It is the right tool for the expensive-to-debug
layer, and it already contains A\*, the FSM, the grid map and the package validator.

Extensions needed to make it predictive of real behaviour:

| Add | Why |
|---|---|
| Waypoint-graph planner alongside the grid planner | Validate §7.1 against your existing A\* as ground truth |
| Ultrasonic sensor model: 15° cone, 4 m max, 20 cm min, ±1 cm noise, **misses glass and angled walls** | Without modelled blind spots, the sim gives false confidence. Model the failures, not just the successes. |
| Odometry noise model: 3% distance error, 0.5°/s gyro drift, wheel slip on turns | Makes drift visible in simulation so §7.3 is validated before hardware exists |
| The 20-state FSM from §8.2 with real timeouts | `ztest/test_statemachine.py` becomes the firmware spec |
| The §5 packet protocol as a Python module | Reused directly by the HIL bridge (§10.4) — write it once |

**Cost:** free. **Effort:** ~1–2 weeks of the work you're already doing.

### 10.2 Layer 2 — Firmware & peripherals: Wokwi + Proteus/SimulIDE

| Tool | Best for | Limitations | Cost |
|---|---|---|---|
| **[Wokwi](https://wokwi.com)** ⭐ | Runs your **actual compiled AVR binary**. Excellent I2C LCD, 4×4 keypad, HC-SR04 and logic-analyser support. Ideal for building and debugging the entire LCD menu tree and PIN entry flow before any hardware arrives. | No ATmega1284P (use **Arduino Mega** as the stand-in — same core, 2 UARTs). Awkward for two boards in one project: simulate each node separately with the peer's packets injected on the serial console. | Free |
| **Proteus 8 VSM** | The only tool that co-simulates **both MCUs plus the analogue power stage** on one virtual schematic — two AVRs with a virtual UART between them, L298N/motor models, LCD, keypad. Doubles as your schematic capture and PCB layout. | Paid, Windows-only. Ultrasonic needs a library part or a manual echo generator. Universities often have a licence — check first. | Paid |
| **[SimulIDE](https://simulide.com)** | Free, cross-platform alternative to Proteus. Multi-MCU AVR simulation, LCD, keypad, DC motor with encoder, serial terminals. Good enough to prove the two-MCU protocol handshake. | Fewer models, less polished, occasional AVR peripheral gaps. | Free |
| **Tinkercad Circuits** | Zero-install, good for teaching a teammate the LCD/keypad/HC-SR04 basics in an afternoon. | Uno only, one board, no reliable two-MCU UART. Not a project tool. | Free |

**Recommended combination:** Wokwi for firmware logic and the HMI (the majority of your firmware
lines), plus SimulIDE (or Proteus if licensed) purely to prove the two-MCU UART link and the motor
driver interface.

### 10.3 Layer 3 — Navigation behaviour (optional): Webots

**[Webots](https://cyberbotics.org)** — free, open source, cross-platform. Has a built-in
differential-drive robot, distance-sensor nodes you can configure as a 5-sonar ring with realistic
cones, and lets you build the actual corridor layout as a 3D world. Use it to validate the §7.4
rerouting ladder and the §7.3 junction-detection logic against realistic geometry and moving
pedestrian obstacles — things the 2D Python sim shows less convincingly.

**Explicitly do NOT use Gazebo + ROS.** It is heavier, steeper, and the moment ROS is in the
project someone will suggest running it on a Pi. Webots gives you the 90% that matters with none
of that pull. Treat this layer as optional: skip it if the schedule is tight.

### 10.4 Layer 4 — Hardware-in-the-loop ⭐ highest value per hour

This is the technique that will save the most time, and it exists only because you already have a
Python simulator.

```
┌────────────────────────┐   USB-TTL adapter   ┌──────────────────────────┐
│  PYTHON SIMULATOR      │◄───── UART ────────►│  REAL ATmega1284P        │
│  (this repo)           │      38400 8N1      │  running REAL firmware   │
│                        │                     │                          │
│  - impersonates the    │  §5 packet protocol │  - real FSM              │
│    Arduino sensor node │  ─────────────────► │  - real planner          │
│  - simulated corridors │                     │  - real PID              │
│    and moving people   │                     │  - real LCD & keypad     │
│                        │                     │    (physically wired)    │
│  - reads motor PWM     │  ◄───────────────── │  - real PWM outputs      │
│    duty via a second   │   duty cycle via    │                          │
│    USB-TTL or an ADC   │   debug telemetry   │                          │
│  - moves the virtual   │   on USART1         │                          │
│    robot accordingly   │                     │                          │
│  - renders it live     │                     │                          │
└────────────────────────┘                     └──────────────────────────┘
```

The real firmware makes real decisions against a simulated world. You can test the whole obstacle
ladder, a total sensor failure, a 250 ms link timeout, a blocked corridor and a low battery — in
minutes, repeatably, with no risk of driving a 5 kg robot into a wall. Then you flip one wire from
the USB-TTL to the real Arduino and the same firmware runs the physical robot.

**Implementation cost is low** because the packet codec (§10.1) is shared between the simulator and
the bridge. Budget 2–3 days. It will pay back within the first week of integration.

### 10.5 Summary

| Layer | Tool | Priority | Cost |
|---|---|---|---|
| Algorithms, FSM, planner, map | **Python sim (this repo)** | Essential | Free |
| Firmware + LCD/keypad/sonar peripherals | **Wokwi** | Essential | Free |
| Two-MCU UART link + motor stage | **SimulIDE** or Proteus | Recommended | Free / Paid |
| Integration & failure injection | **HIL bridge** | High value | ~3 days |
| 3D navigation behaviour | Webots | Optional | Free |
| Beginner onboarding | Tinkercad | Optional | Free |

---

## 11. Development roadmap

Six phases. Each has a **hard exit criterion** — do not begin the next phase until the current one
demonstrably passes. Durations assume a small team working part-time alongside coursework; compress
by parallelising across members, not by skipping exit criteria.

### Phase 0 — Specification freeze & building survey (1 week)

| Task | Owner | Deliverable |
|---|---|---|
| Survey the actual corridor with a tape measure: lengths, widths, every doorway and opening | Navigation | `map/floor1_survey.csv` committed |
| Build the waypoint graph from the survey; generate `map_data.h` | Navigation | ~40 nodes, ~50 edges in PROGMEM format |
| **Freeze the §5 packet protocol** and commit it as `protocol.h` + `protocol.py` | Lead | Both teams unblocked permanently |
| Freeze the §8.2 FSM state and transition table | Lead | Ported into `core/state_machine.py` |
| Confirm weight limit, speed, thresholds in one config header | Lead | `config.h` mirroring `config/settings.py` |
| Place the parts order (§3, core groups only — **not** the optional group) | Hardware | 2–3 week lead time starts now |

**Exit:** protocol and FSM committed and reviewed; parts ordered; the survey has real measured
numbers, not estimates from a floor plan PDF.

> Order parts in Phase 0. The single most common capstone failure mode is a team that finishes the
> software in week 8 and is still waiting on motors in week 10.

### Phase 1 — Python simulation upgrade (2 weeks, parallel with parts shipping)

| Task | Exit criterion |
|---|---|
| Implement the waypoint-graph planner | Produces the same routes as the existing grid A\* on 20 test cases |
| Add the ultrasonic sensor model with blind spots | Sim shows the robot failing on glass — good, that's the point |
| Add the odometry noise model (3% distance, 0.5°/s drift, turn slip) | Drift is visible and accumulates realistically |
| Implement junction detection and snapping (§7.3) | Position error stays under 40 cm over a 30 m route |
| Implement the full §7.4 obstacle ladder | All 6 rungs demonstrably exercised by scripted scenarios |
| Replace QR with PIN; replace dimension entry with the physical-bay assumption | `test_delivery.py` updated and green |
| Port the 20-state FSM; expand `test_statemachine.py` | Every transition and every timeout covered by a test |
| Write `protocol.py` (encode/decode/CRC8) with round-trip tests | Byte-identical to the `protocol.h` spec |

**Exit:** 20 consecutive simulated deliveries succeed with noise and moving obstacles enabled, and
the failure scenarios (blocked route, sensor fault, lost position) all terminate in the correct
state rather than hanging.

### Phase 2 — Arduino sensor node, standalone (1.5 weeks)

Build only the sensor node. No motors, no chassis, no brain. Breadboard on a desk.

| Task | Exit criterion |
|---|---|
| Wire 5× HC-SR04 per §4.2 (all echoes on PORTB) | All five read plausible distances |
| PCINT-based echo capture, **no `pulseIn()`** | Loop iteration time stays under 2 ms during a sweep |
| Implement the §5.5 sweep schedule | Verified with a logic analyser or Wokwi timing view: slots are 50 ms apart |
| Median filter + fault detection | Waving a hand produces smooth mm output; unplugging a sensor produces `0x0000` and a `SENSOR_FAULT` |
| `SENSOR_TELEMETRY` framing + CRC8 | A Python script on a laptop decodes the stream with zero CRC errors over 10 minutes |
| `EMERGENCY_OUT` line | Scope-verified to assert within 10 ms of an object entering 200 mm |
| Build the LCD + keypad + HX711 menu tree in **Wokwi** in parallel | Full destination-entry and PIN-entry flow works in simulation |
| Calibrate the load cell with the 2.000 kg reference | Reads within ±20 g across 0–3 kg |

**Exit:** the node streams valid, filtered telemetry to a laptop for 30 minutes with no CRC errors
and no lock-ups, and the Wokwi HMI flow is complete.

### Phase 3 — Arduino-as-brain rolling prototype (2 weeks) ⭐ de-risking phase

**Use the second Arduino Uno as a temporary brain.** This is the most important scheduling decision
in the roadmap: it separates "does my robot drive?" from "does my bare ATmega boot?" — two hard
problems that are miserable to debug simultaneously.

| Task | Exit criterion |
|---|---|
| Assemble the chassis, motors, wheels, casters, cargo bay | Rolls freely; cargo bay sits on the load cell without binding |
| Wire the power architecture per §9 including fuse, E-stop, relay, star ground | Multimeter check: 5 V rail holds ≥4.8 V during a motor stall |
| Fit the motor EMI capacitors **now, during assembly** | No MCU resets when motors start |
| Encoder reading + odometry | Commanded 2.00 m travel lands within ±5 cm, repeatable over 10 trials |
| Wheel velocity PID | Holds 0.30 m/s ±0.02 m/s with and without a 2 kg load |
| Gyro-closed-loop 90° turn | Within ±3°, repeatable over 20 turns |
| Corridor centring from side sonars | Robot self-centres in a real corridor and holds within ±5 cm over 8 m |
| Junction detection on a real corridor | Detects every real opening; zero false positives over 10 runs |
| Full navigation on a **taped-out test track** | Reaches 3 different destinations, 10 runs each |
| Obstacle ladder rungs 1–4 on the real robot | Person steps in front → slow, stop, wait, resume. Then sidestep. Then reroute. |
| Safety: bumper stop, E-stop, link timeout, brownout | Each verified by deliberately triggering it |

**Exit:** a fully working autonomous delivery robot — running on an Arduino brain. Every mechanical,
electrical, tuning and safety problem is now solved. Only the chip changes from here.

> If the schedule collapses, **this is a demonstrable, defensible project on its own.** Phase 4 is
> the part that satisfies the "standalone ATmega" requirement, and it should be attempted from a
> position of strength, not desperation.

### Phase 4 — Port the brain to a standalone ATmega1284P (1.5 weeks)

| Task | Exit criterion |
|---|---|
| Install **MightyCore** in the Arduino IDE/CLI | ATmega1284P board target available |
| Build the minimum circuit on perfboard: socket, 16 MHz crystal + 2× 22 pF, reset pull-up, decoupling, ISP header | Continuity-checked before power |
| Set up **Arduino-as-ISP** using the Phase-3 Uno | Reads the signature `0x1E 0x97 0x05` |
| Burn fuses: external crystal (CKSEL/SUT), **BOD 4.3 V**, preserve EEPROM, do **not** disable the reset pin | Blink sketch runs at the correct speed (verify with a 1 Hz LED against a stopwatch) |
| Bring up USART1 debug console first, before anything else | `printf`-style logging works — you are no longer blind |
| Port peripherals one at a time, testing each: I2C/LCD → keypad → HX711 → encoders → PWM → USART0 link | Each subsystem individually verified before adding the next |
| Port the FSM, planner, PID (source is unchanged from Phase 3 — only pin macros and timer setup differ) | Re-run the full Phase-3 test suite |
| Verify free SRAM and check for stack overflow headroom | ≥8 KB free after a worst-case A\* call |

**Exit:** the standalone ATmega1284P board passes every Phase-3 exit criterion. Then physically
remove the Arduino brain from the robot.

> **Fuse-bit warning:** setting CKSEL to external crystal when no crystal is fitted makes the chip
> unresponsive to ISP. Recovery requires injecting an external clock on XTAL1 or a
> high-voltage/parallel programmer. Fit the crystal **before** burning fuses, and this is why the
> BOM has two chips.

### Phase 5 — Integration, hardening & real-corridor trials (2 weeks)

| Task | Exit criterion |
|---|---|
| Build the HIL bridge (§10.4) | All failure scenarios injectable and repeatable |
| HIL regression: link loss, sensor fault, blocked route, low battery, lost position, PIN lockout | Every case terminates in the correct state; none hangs |
| 50 consecutive deliveries on the real corridor | ≥45 succeed unaided; **zero collisions with a person** (non-negotiable) |
| Battery endurance run | ≥2 h of continuous mixed-duty operation |
| Cable management, strain relief, enclosure, labelled connectors | Survives being carried up a flight of stairs and still works |
| Fault-code card and operator instructions taped to the robot | A stranger can recover it from `PARK_ALERT` |
| Demo script with a deliberate obstacle and a deliberate overweight package | Rehearsed 3 times end to end |
| Documentation: schematic, pin map, protocol spec, calibration procedure, this document updated to match what was actually built | Committed |

**Exit:** demo-ready, with a rehearsed script and a fallback plan for each failure mode.

### Phase 6 — Optional experiments (only if Phase 5 is complete)

| Experiment | Precondition |
|---|---|
| ESP32-CAM vision hint flags (§4.5) | 45/50 delivery success already achieved |
| Edge-cost inflation instead of hard blocking (§7.4) | Rerouting already works with hard blocking |
| Multi-floor in simulation only | Never on hardware |
| Custom PCB replacing the perfboard | Only if a spare 2 weeks exists |

### Schedule summary

| Phase | Duration | Cumulative | Can run in parallel with |
|---|---|---|---|
| 0 — Spec freeze & survey | 1 wk | 1 wk | — |
| 1 — Python simulation | 2 wk | 3 wk | Parts shipping |
| 2 — Arduino sensor node | 1.5 wk | 4.5 wk | Tail of Phase 1; Wokwi HMI work |
| 3 — Arduino-brain prototype | 2 wk | 6.5 wk | — (all hands) |
| 4 — Standalone ATmega port | 1.5 wk | 8 wk | HIL bridge development |
| 5 — Integration & trials | 2 wk | **10 wk** | Documentation |
| 6 — Optional | — | — | Only after Phase 5 |

Ten weeks with two weeks of slack inside a typical 12–14 week semester. The slack is real and you
will use it — most likely on odometry tuning in Phase 3.

---

## 12. Technical risks, limitations, and what NOT to add

### 12.1 Risk register

Ordered by expected impact on the project, not by likelihood.

| # | Risk | Likelihood | Impact | Mitigation (designed in) |
|---|---|---|---|---|
| R1 | **Odometry drift** — robot loses track of its position and stops at the wrong door or drives into a wall | **High** | **High** | Legs capped at 8 m; gyro heading hold; side-sonar corridor centring; junction snapping zeroes error at every node (§7.3); `RECOVERY` 360° scan. Budget the most tuning time here — it *is* the project's core difficulty. |
| R2 | **Ultrasonic blind spots** — glass doors, walls at a shallow angle (specular reflection away from the sensor), soft/fabric surfaces, thin chair legs, objects below sensor height | **High** | **High** | 5-sensor spread; 300 mm stop threshold (2× stopping distance); 0.3 m/s cruise; **front bumper microswitches as the physical last resort**; route selection avoids glass-walled corridors. Accept and document that HC-SR04 cannot see everything — this is an inherent limitation, not a bug. |
| R3 | **Motor EMI resets the MCU** | High if unmitigated | High | 100 nF across motor terminals and to case; 1000 µF bulk at each driver; star ground; physical separation of motor and signal wiring; BOD at 4.3 V; watchdog. Do all of this during assembly, not as a fix. |
| R4 | **Battery sag under stall causes brownout** | Medium | High | 10 A fuse; BMS; buck with headroom; BOD + watchdog; reset state is motors-off by hardware; current sense on `IS` pins detects a stall before the battery collapses. |
| R5 | **Standalone ATmega bring-up failure** (wrong fuses, missing crystal caps, ISP miswiring) | Medium | High | Phase 3 proves the robot on an Arduino first; second 1284P in the BOM; fuses burned only after the crystal is fitted; USART1 debug console brought up before any other peripheral. |
| R6 | **Load cell misbehaviour** — mechanical binding, thermal drift, wrong tare, reading changes when the robot moves | Medium | Medium | Cantilever mount with a real deflection gap; tare on every `IDLE` entry; 10-sample stability gate before accepting; two casters so the platform doesn't rock; validate with the 2.000 kg reference in every test session. |
| R7 | **Serial link corruption / desync** | Medium | Medium | CRC8 + sync bytes + auto-resync parser; 38400 at 0.2% clock error; 250 ms timeout → `SAFE_STOP`; `seq` counter exposes drop rate in the debug log; **common ground** (check this first when it misbehaves). |
| R8 | **Human factors** — people blocking corridors, closed doors, wet floors, someone picking the robot up | Medium | Medium | Wait-first ladder (§7.4); `PARK_ALERT` needs a human; buzzer while moving; door-closed is indistinguishable from a blockage and correctly ends at reroute/abort. |
| R9 | **SRAM exhaustion / stack overflow** | Medium | High (silent, random resets) | 1284P's 16 KB; no `String`, no heap; `PROGMEM` tables; stack-painting free-RAM reporting in the debug log; watchdog reason latched across reset. |
| R10 | **Torque/traction underestimated** — cannot cross a door sill or a carpet-to-tile transition | Medium | Medium | 3.3× stall margin (§3.9); rubber wheels; **test with a 2 kg dummy load in the first week of Phase 3**; if it fails, route around the transition rather than upgrading motors. |
| R11 | **CV scope creep consumes the schedule** | **High** | **High** | Camera is a 3-GPIO flag with a documented no-effect-when-absent property (§4.5), and Phase 6 is gated on 45/50 delivery success. This is a schedule-management risk more than a technical one. |
| R12 | **Parts arrive late** | Medium | High | Order in Phase 0; Phases 1–2 need almost no hardware; Wokwi covers HMI development with zero parts. |
| R13 | **Cargo shifts / robot tips** | Low | Medium | Battery low and central (lowest centre of gravity); enclosed 16 cm bay; 0.3 m/s and gentle acceleration limits; two casters for a stable rectangular footprint. |
| R14 | **I2C bus lock-up** (a stuck slave holds SDA low) | Low | Medium | Bus-recovery routine (9 manual SCL clocks) on timeout; short bus routed away from motor wiring; LCD failure is detected in `SELF_TEST`. |
| R15 | **Team knowledge concentration** — one person understands the ATmega | Medium | High | Freeze the protocol in Phase 0 so the sensor node and brain are independently ownable; require a documented pin map and schematic; `SELF_TEST` fault codes let anyone triage. |

### 12.2 Accepted limitations (state these openly in your report)

Naming your limitations is stronger engineering than pretending they don't exist, and examiners
consistently reward it.

| Limitation | Consequence |
|---|---|
| No SLAM, no map building | Only operates on a pre-surveyed route. Move the robot to a new building and it needs a new survey. |
| Position is dead-reckoned with topological fixes | Accurate to roughly ±10–40 cm within a corridor. Adequate for stopping at a door, not for precise docking. |
| Ultrasonic-only obstacle detection | Cannot classify obstacles; cannot see glass, thin legs, or overhangs above/below the sensor plane. Detects presence, not identity. |
| Single floor | No stairs, no elevator. Multi-floor exists only in the simulator. |
| Robot stops outside the room | Does not enter rooms or hand a package to a specific desk. |
| 2 kg / 16 cm cargo limit | Documents, small parcels, lab samples. Not general logistics. |
| PIN, not QR | Weaker than a cryptographic token; a PIN can be shared or overheard. Acceptable for a campus courtesy system, and it is honest about being so. |
| 0.3 m/s cruise | A 100 m delivery takes ~6 minutes. Slow by design — a faster robot needs faster sensing than HC-SR04 can provide. |
| Wheeled, not legged | The repo is named "robodog" but the platform is a wheeled differential-drive robot (§12.4). |
| No fleet coordination, no networking | One robot, one delivery at a time, no remote monitoring. |

### 12.3 Do NOT add these — and why

| Do not add | Why it breaks this project |
|---|---|
| **Raspberry Pi / Jetson** | Violates the core premise. The moment a Pi exists, the ATmega becomes a motor driver and the "ATmega as decision-making brain" claim — the actual thesis of your project — evaporates. |
| **LiDAR + SLAM** | RPLIDAR A1 costs more than half your entire BOM and produces ~8000 points/s. An AVR cannot process that, let alone run a particle filter or graph SLAM. There is no partial version of this that works. |
| **YOLO / any neural network on-board** | 128 KB flash, 16 KB SRAM, 16 MHz. Not "slow" — architecturally impossible. Even an ESP32-CAM manages perhaps 1–2 FPS of crude person detection, which is why §4.5 makes it a *hint*, not a sensor. |
| **ROS / ROS2** | Needs a Linux host. Adds weeks of learning for zero benefit at 40 nodes and 50 edges. It is also the trojan horse that reintroduces the Pi. |
| **4WD or mecanum wheels** | Doubles motor count, current draw and cost. Skid-steering *actively destroys* odometry accuracy through slip — it attacks your single highest risk (R1). Mecanum wheels slip constantly by design and are the worst possible choice for dead reckoning. |
| **Servo cargo lid / robot arm / auto-loading** | Servos under load draw 0.5–1 A each, add mechanical failure modes, and solve a problem you don't have: a human is standing right there and can open a lid. |
| **Wi-Fi / Bluetooth remote control app** | Direct threat to your grade. Any control channel invites "so is it actually remote-controlled?" from your examiners. If you want remote *monitoring*, make it strictly one-way telemetry, on a separate ESP8266, with no command path — and be ready to justify it. |
| **GPS** | Zero indoor signal. Non-functional. |
| **Elevator interaction / stair climbing** | Requires actuating building infrastructure or a fundamentally different chassis. Both are multi-month projects. Keep them in the simulator's "future work". |
| **Legged / quadruped locomotion** | 8–12 servos, inverse kinematics, gait generation, a much bigger battery, and a chassis that cannot reliably carry 2 kg. This is a whole capstone on its own. See §12.4. |
| **Camera for QR scanning** | The reason the PIN exists. Barcode decoding needs image processing the AVR cannot do. |
| **Auto-docking charge contacts** | Requires sub-centimetre docking precision — well beyond your ±10–40 cm localisation. Plug it in by hand. |
| **A second robot / fleet management** | Multiplies every hardware risk by two before one robot works reliably. |
| **Cliff/edge IR sensors** | Only if the route has an actual unguarded drop. On a flat single-floor corridor they are two more things to debug for no benefit. |
| **Temperature/humidity/gas sensors, RTC, SD logging** | Feature padding. If it doesn't serve "get this package to room 201 safely", it costs pins, current and debugging time. Log over USART1 to a laptop instead of adding an SD card. |

### 12.4 One honest note on the "robodog" name

The repo is called `robodog-ai` and the README describes a robotic dog. **The architecture in this
document is a wheeled differential-drive robot, and that is the correct engineering decision.**

| | Wheeled (this design) | Quadruped |
|---|---|---|
| Actuators | 2 motors | 8–12 servos |
| Extra cost | — | +$120–250 |
| Payload at 2 kg | Comfortable | Very difficult |
| Odometry | Encoder-based, works | Foot-slip dominated, effectively unusable without an IMU-heavy state estimator |
| Control complexity | PID on 2 wheels | Inverse kinematics + gait generation + balance |
| Realistic for one semester | Yes | No |

If the "dog" identity matters for your presentation, **put a dog-styled shell over the wheeled
chassis.** You keep the branding, the demo looks the part, and you spend your semester on
navigation and decision-making — which is what a Computer Engineering project should actually
demonstrate. State the reasoning explicitly in your report; deliberately choosing the simpler
mechanism for a defensible reason is an engineering result, not a compromise.

Alternatively, rename the project to match what it is. "Autonomous Indoor Delivery Robot" describes
the system accurately and sets no expectations you then have to manage.

---

## 13. Quick reference

### 13.1 Key parameters in one place

| Parameter | Value |
|---|---|
| Main MCU | ATmega1284P, 16 MHz external crystal, BOD 4.3 V, WDT 1 s |
| Sensor MCU | ATmega328P (Arduino Uno/Nano), 16 MHz |
| Inter-MCU link | UART 38400 8N1, framed + CRC8, 20 Hz telemetry, 250 ms timeout |
| Ultrasonic sweep | 5 sensors, 4 slots × 50 ms = 5 Hz |
| Cruise speed | 0.30 m/s (49% motor duty) |
| Manoeuvre speed | 0.15 m/s |
| Stop threshold | 300 mm (front) |
| Slow threshold | 600 mm (front) |
| Hardware emergency threshold | 200 mm (front, bypasses firmware) |
| Stopping distance @0.3 m/s | ~147 mm |
| Wheel diameter / circumference | 65 mm / 204.2 mm |
| Track width | 200 mm |
| Encoder resolution | 1232 counts/rev = 0.166 mm/count |
| Max package weight | 2000 g (5 kg load cell) |
| Cargo bay internal | 160 × 160 × 160 mm (= the dimension validator) |
| Delivery PIN | 6 digits, 3 attempts, 60 s lockout |
| Max leg length | 8 m |
| Battery | 3S Li-ion, 11.1 V nominal, 6.8 Ah, BMS |
| Motor PWM | 7.81 kHz, 8-bit, Timer1 + Timer2 |
| Control rates | 100 Hz PID/odometry · 50 Hz FSM/arbitration · 20 Hz telemetry+waypoints · 10 Hz LCD/HX711 · 2 Hz battery |
| Runtime | ~2.5–3 h mixed duty |
| Core BOM cost | ≈ $223 (≈ IDR 3.6 M) |

### 13.2 Fault codes (print these on a card and tape it to the robot)

| Code | Meaning | First thing to check |
|---|---|---|
| E01 | I2C device missing | LCD/keypad/IMU wiring, 4.7 kΩ pull-ups |
| E02 | LCD not responding | Backpack address (0x27 vs 0x3F), contrast pot |
| E03 | Keypad fault / key stuck | Ribbon seating, expander address 0x20 |
| E04 | HX711 not responding | DT/SCK wiring, 5 V at the module |
| E05 | Arduino link dead | **Common ground**, TX/RX crossed, debug jumper refitted? |
| E06 | Front ultrasonic faulted | Which sensor (`valid_mask` in the log), 5 V, echo wire |
| E07 | Encoder fault or reversed | A/B channels swapped on that motor |
| E08 | Battery too low | Charge it |
| E09 | Bumper stuck closed | Mechanical obstruction on the bumper |
| E10 | Motor relay fault | Relay coil driver transistor, flyback diode |
| W01 | Load cell tare drifted | Re-tare with an empty bay |
| W02 | IMU unavailable | Degraded: encoder-only turns, expect worse accuracy |
| W03 | Side ultrasonic faulted | Degraded: no corridor centring, expect lateral drift |

### 13.3 Immediate next steps

1. Survey the corridor and commit the measurements. Everything downstream depends on this.
2. Commit `protocol.h` / `protocol.py` and the 20-state transition table. Freeze both.
3. Place the parts order for §3.1–§3.6. **Skip §3.7 entirely** for now.
4. Update `core/state_machine.py` and `ztest/test_statemachine.py` to the §8.2 model.
5. Write `tools/grid_to_graph.py` so the existing map work feeds the firmware map header.
6. Start the LCD + keypad + PIN-entry flow in Wokwi today — it needs no hardware at all.
