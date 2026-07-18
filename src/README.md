# 🐭 Toshiba Micromouse — Firmware

> **Work in Progress** — Firmware for an IEEE 16×16 micromouse

```
    ╔══════════════════════════════════════════╗
    ║  ┌─────────┐    ┌─────────┐              ║
    ║  │  LEFT   │    │  RIGHT  │              ║
    ║  │  MOTOR  │    │  MOTOR  │              ║
    ║  │  T32A0  │    │  T32A3  │              ║
    ║  └────┬────┘    └────┬────┘              ║
    ║       │              │                   ║
    ║  ┌────┴──────────────┴────┐              ║
    ║  │    TMPM4KNF10AFG       │              ║
    ║  │    Cortex-M4 @ 160MHz  │              ║
    ║  │    ┌───────────────┐   │              ║
    ║  │    │  1kHz Control │   │              ║
    ║  │    │  Loop (T32A1) │   │              ║
    ║  │    └───────────────┘   │              ║
    ║  └────────────────────────┘              ║
    ║       │              │                   ║
    ║  ┌────┴────┐    ┌────┴────┐              ║
    ║  │  ENC0   │    │  ENC2   │              ║
    ║  │  Left   │    │  Right  │              ║
    ║  └─────────┘    └─────────┘              ║
    ╚══════════════════════════════════════════╝
```

---

## Architecture

Everything is driven by a single 1 kHz control tick. Each tick runs four stages
in order; each stage consumes the output of the one above it. The maze planner
(`FloodFill`) is pure logic and never touches hardware — `Navigator` owns all
motion and calls the planner for decisions.

```
┌─────────────────────────────────────────────────────────────┐
│                          main.c                             │
│   while(1) {                                                │
│     if (Timebase_GetAndClear()) {   ◄── 1 kHz tick          │
│       Encoder_Update();    // read encoders, filter speed   │
│       Odometry_Update();   // pose (x, y, heading)          │
│       Motion_Update();     // PID -> motor PWM (never block)│
│       Navigator_Update();  // plan/turn/drive FSM step      │
│     }                                                       │
│   }                                                         │
└──────────────────────────┬──────────────────────────────────┘
                           │  once per tick
   ┌───────────┬───────────┼───────────┬────────────┐
   ▼           ▼           ▼           ▼            ▼
┌────────┐ ┌────────┐ ┌────────┐ ┌───────────┐ ┌──────────┐
│Encoder │ │Odometry│ │ Motion │ │ Navigator │ │ FloodFill│
│speed + │ │pose    │ │PID +   │ │ FSM:      │◄┤ pure BFS │
│position│ │x,y,θ   │ │motor   │ │ plan→turn │ │ planner  │
└───┬────┘ └───┬────┘ └───┬────┘ │ →drive    │ └────┬─────┘
    │          │          │      └─────┬─────┘      │
    ▼          │          ▼            │            ▼
┌────────┐     │     ┌─────────┐       │       ┌──────────┐
│ENC0/2  │     │     │ Motor   │       │       │ IrSensor │
│A-ENC32 │     └────►│ TB67H450│       └──────►│ ADC+DMA  │
│quad in │  (via     │ +T32A0/3│  (checks pose │ 4× wall  │
└────────┘  Encoder) │ PWM PPG │   for arrival)│ sensing  │
                     └─────────┘               └──────────┘
```

**Control flow summary**

| Stage | Module | Role |
|-------|--------|------|
| 1 | `Encoder` | Read A-ENC32 counters, IIR-filter wheel speed (CPS) |
| 2 | `Odometry` | Integrate differential-drive pose from encoder deltas |
| 3 | `Motion` | Per-wheel PID speed loop → `Motor` PWM duty |
| 4 | `Navigator` | Non-blocking FSM; asks `FloodFill` for moves, executes them |
| — | `FloodFill` | Pure BFS planner over discovered walls (no hardware) |
| — | `IrSensor` | 4× IR wall detection via ADC + DMA (used by planner) |

---

## Project Structure

```
toshiba-micromouse/
├── keil/                     # Keil µVision project files
├── pcb/                      # 4-layer KiCad PCB design files
├── docs/                     # Images and documentation assets
├── README.md                 # Project landing page
└── src/
    ├── README.md             # This file — firmware architecture
    ├── main.c                # Entry point, 1 kHz control loop
    │
    ├── drivers/              # Register-level hardware layer
    │   ├── timer32A.c/h      # T32A: PWM (ch0/3) + 1 kHz interval (ch1)
    │   ├── gpio.c/h          # Port configuration + emitter GPIO
    │   ├── encoder32A.c/h    # A-ENC32 quadrature hardware read
    │   ├── adc.c/h           # ADC-I units A & C (IR receivers)
    │   ├── dma.c/h           # DMAC-B burst transfer for ADC
    │   ├── uart.c/h          # UART-C serial debug output
    │   └── systick.c/h       # Blocking µs/ms delay (stateless)
    │
    └── modules/              # Application logic layer
        ├── Timebase.c/h      # 1 kHz tick flag service
        ├── Encoder.c/h       # Filtered speed + signed position
        ├── Odometry.c/h      # Differential-drive pose (x, y, θ)
        ├── PID.c/h           # Generic PID controller math
        ├── Motor.c/h         # TB67H450 H-bridge direction + duty
        ├── Motion.c/h        # PID speed loop bridging Encoder→Motor
        ├── IrSensor.c/h      # IR sampling, ambient cancel, distance
        ├── FloodFill.c/h     # BFS flood-fill maze planner
        └── Navigator.c/h     # Cell-level motion sequencer (FSM)
```

---

## Hardware Specs

| Component | Part | Interface |
|-----------|------|-----------|
| **MCU** | TMPM4KNF10AFG | Arm Cortex-M4, 160 MHz, 5 V |
| **Motors** | TB67H450AFNG + Pololu #5211 N20 30:1 | T32A PPG PWM @ 40 kHz |
| **Encoders** | A-ENC32 (on-chip) | Quadrature, 12 CPR × 29.89 ≈ 360/rev |
| **IR Sensors** | IR LED + phototransistor ×4 | ADC-I + DMA |
| **Debug** | CMSIS-DAP + Level Shifter (TXB0104) | SWD |

### Pin Map

```
    ┌─────────────────────────────────────────────┐
    │  PORT A  │ PA3  ──► T32A00OUTA  (Left PWM)  │
    │          │ PA4  ──► T32A00OUTB  (Left PWM)  │
    ├─────────────────────────────────────────────┤
    │  PORT C  │ PC2  ──► T32A30OUTA  (Right PWM) │
    │          │ PC3  ──► T32A30OUTB  (Right PWM) │
    ├─────────────────────────────────────────────┤
    │  PORT N  │ PN0  ──► ENC0A  (Left encoder)   │
    │          │ PN1  ──► ENC0B  (Left encoder)   │
    ├─────────────────────────────────────────────┤
    │  PORT D  │ PD3  ──► ENC2A  (Right encoder)  │
    │          │ PD4  ──► ENC2B  (Right encoder)  │
    ├─────────────────────────────────────────────┤
    │  PORT L  │ PL0  ──► AINA16  (Far Left IR)   │
    │          │ PL1  ──► AINA15  (Left IR)       │
    ├─────────────────────────────────────────────┤
    │  PORT J  │ PJ0  ──► AINC00  (Far Right IR)  │
    │          │ PJ1  ──► AINC01  (Right IR)      │
    ├─────────────────────────────────────────────┤
    │  PORT U  │ PU0  ──► Left IR Emitter         │
    │          │ PU1  ──► Far Left IR Emitter     │
    ├─────────────────────────────────────────────┤
    │  PORT G  │ PG4  ──► Far Right IR Emitter    │
    │          │ PG5  ──► Right IR Emitter        │
    └─────────────────────────────────────────────┘
```

---

## Timer Configuration

```
┌──────────────────────────────────────────────────────────────┐
│  T32A0 (Left Motor)     │  T32A3 (Right Motor)               │
│  ─────────────────────  │  ─────────────────────             │
│  Mode:     16-bit PPG   │  Mode:     16-bit PPG              │
│  Prescaler: 1:1         │  Prescaler: 1:1                    │
│  Frequency: 40 kHz      │  Frequency: 40 kHz                 │
│  Period:   2000 counts  │  Period:   2000 counts             │
│  Pins:     PA3, PA4     │  Pins:     PC2, PC3                │
│  Duty:     RG0 = 0-2000 │  Duty:     RG0 = 0-2000            │
├──────────────────────────────────────────────────────────────┤
│  T32A1 (Control Loop)                                        │
│  ─────────────────────                                       │
│  Mode:     32-bit Interval                                   │
│  Prescaler: 1:1                                              │
│  Frequency: 1 kHz                                            │
│  Period:   80000 counts                                      │
│  Interrupt: INTT32A01AC (NVIC)                               │
│  Flag:     T32A01AC_IRQ_Fire (volatile bool)                 │
└──────────────────────────────────────────────────────────────┘
```

---

## Navigation Model

`FloodFill` and `Navigator` deliberately use **relative** moves at their
boundary, so neither needs to agree on an absolute compass:

- `FloodFill_Plan()` returns one of: `FORWARD`, `TURN_LEFT`, `TURN_RIGHT`,
  `TURN_AROUND`, `STOP`. It tracks its own grid cell + heading in the maze.
- `Navigator` converts each move into a target heading in radians, tracked as
  an **exact grid index** (0/±90°/180°) — never by accumulating odometry — so
  per-turn error does not compound across a run.
- Odometry is used only to *check* when a turn/drive is complete, never to
  *define* the next target.

> **Invariant (load-bearing):** the N→E→S→W cycle must be clockwise in both
> `FloodFill` and `Navigator`'s heading table. Renumbering either breaks turns.

---

## Build & Flash

```text
IDE:     Keil µVision
Target:  TMPM4KNF10AFG
Debug:   CMSIS-DAP
Flash:   On-chip 512 KB
```

---

## Deployment Calibration

Before real runs, these must be measured/tuned on the actual robot:

| Where | What | Why |
|-------|------|-----|
| `Encoder` | Confirm forward → **increasing** position | Reversed sign turns PID into positive feedback (runaway) |
| `Odometry.h` | `WHEELBASE_MM` (wheel-center to center) | Wrong value → every turn over/under-rotates |
| `IrSensor.c` | `ir_cal[]` ADC→distance points | Placeholder values; wall detection unreliable until measured |
| `PID.h` | Gains for CPS-scaled error | Error is ~thousands (CPS); default `Kp` saturates instantly |

---

## Module Status

| Module | Status | Description |
|--------|--------|-------------|
| `Timebase` | Done | 1 kHz control tick flag |
| `Encoder` | Done | Filtered speed + signed position |
| `Odometry` | Done | Differential-drive pose estimate |
| `PID` | Done | Generic PID controller |
| `Motor` | Done | H-bridge direction + duty |
| `Motion` | Done | PID speed loop (Encoder→Motor) |
| `IrSensor` | Done | IR sampling + distance (needs calibration) |
| `UART` | Done | Serial debug output (TX) |
| `FloodFill` | Done | BFS flood-fill planner |
| `Navigator` | Bring-up | Motion FSM; verify on hardware |

---

## References

- [TMPM4KNF10AFG Product Page](https://toshiba.semicon-storage.com/us/semiconductor/product/microcontrollers/txz4aplus-series/m4k-group.html)
- [RM-T32A-C Timer Reference](https://toshiba.semicon-storage.com/info/RM-T32A-C_en_20241129.pdf)
- [RM-A-ENC32-A Encoder Reference](https://toshiba.semicon-storage.com/info/RM-A-ENC32-A_en_20250221.pdf)
- [RM-ADC-I Reference](https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf)
- [RM-DMAC-B DMA Reference](https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf)
- [Pololu #5211 Motor Datasheet](https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-2.pdf)

---

**Author:** Kevin Le &nbsp;•&nbsp; 2026