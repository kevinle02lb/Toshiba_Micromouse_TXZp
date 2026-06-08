# 🐭 Toshiba TMPM4KNF10AFG Micromouse

> **Work in Progress** — Firmware for the 2026 All-Japan Micromouse Competition

```
    ╔══════════════════════════════════════════╗
    ║  ┌─────────┐    ┌─────────┐            ║
    ║  │  LEFT   │    │  RIGHT  │            ║
    ║  │  MOTOR  │    │  MOTOR  │            ║
    ║  │  T32A0  │    │  T32A3  │            ║
    ║  └────┬────┘    └────┬────┘            ║
    ║       │              │                 ║
    ║  ┌────┴──────────────┴────┐            ║
    ║  │    TMPM4KNF10AFG        │            ║
    ║  │    Cortex-M4 @ 160MHz   │            ║
    ║  │    ┌───────────────┐    │            ║
    ║  │    │  1kHz Control │    │            ║
    ║  │    │  Loop (T32A1)  │    │            ║
    ║  │    └───────────────┘    │            ║
    ║  └──────────────────────────┘            ║
    ║       │              │                 ║
    ║  ┌────┴────┐    ┌────┴────┐            ║
    ║  │  ENC0   │    │  ENC2   │            ║
    ║  │  Left   │    │  Right  │            ║
    ║  └─────────┘    └─────────┘            ║
    ╚══════════════════════════════════════════╝
```

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         main.c                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  while(1) {                                         │    │
│  │    if (CtrlTick_GetAndClear()) {  ◄── 1kHz tick   │    │
│  │      // Control logic (PID, maze solver)           │    │
│  │    }                                                │    │
│  │  }                                                  │    │
│  └─────────────────────────────────────────────────────┘    │
└──────────────────────────┬──────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
   ┌─────────┐       ┌──────────┐       ┌──────────┐
   │ ctrlTick│       │ motorCtrl│       │ mazeAlg  │
   │ (1kHz)  │       │ (PWM)    │       │ (search) │
   └────┬────┘       └────┬─────┘       └──────────┘
        │                 │
        ▼                 ▼
   ┌─────────┐       ┌─────────┐
   │ T32A1   │       │ T32A0/3 │
   │ Interval│       │ PPG PWM │
   │ Timer   │       │ 40kHz   │
   └─────────┘       └─────────┘
```

---

## 📁 Project Structure

```
TOSHIBAMICRO/
├── 📂 keil/                    # Keil µVision project files
├── 📂 pcb/                     # PCB design files (Altium/KiCad)
├── 📂 src/
│   ├── 📄 main.c               # Entry point, control loop
│   ├── 📄 ctrlTick.c/h         # 1kHz interval timer service
│   │
│   ├── 📂 drivers/             # Hardware abstraction layer
│   │   ├── 📄 timer32A.c/h   # T32A driver (PWM + interval)
│   │   ├── 📄 gpio.c/h       # Port configuration
│   │   ├── 📄 encoder32A.c/h # Quadrature encoder (A-ENC32)
│   │   ├── 📄 adc.c/h        # IR sensor ADC (DMA)
│   │   ├── 📄 dma.c/h        # DMAC-B for ADC burst
│   │   ├── 📄 systick.c/h    # Blocking delay utility
│   │   └── 📄 ...
│   │
│   └── 📂 modules/             # [PLANNED] Application modules
│       ├── 📄 motorCtrl.c/h  # PID + encoder feedback
│       ├── 📄 mazeAlg.c/h    # Flood-fill / A* solver
│       ├── 📄 irSensor.c/h   # IR distance processing
│       └── 📄 motion.c/h     # Straight, turn, pivot profiles
│
├── 📄 readme.md                # This file
└── 📄 .gitignore
```

---

## ⚡ Hardware Specs

| Component | Part | Interface |
|-----------|------|-----------|
| **MCU** | TMPM4KNF10AFG | ARM Cortex-M4, 160 MHz |
| **Motors** | TB67H450AFNG + Pololu #5211 N20 30:1 | T32A PPG PWM |
| **Encoders** | A-ENC32 (on-chip) | Quadrature, 2-phase |
| **IR Sensors** | IR LED + phototransistor ×4 | ADC-I + DMA |
| **Debug** | CMIS-DAP + Level Shifter | SWD |

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
    │          │ PJ1  ──► AINC01  (Right IR)       │
    ├─────────────────────────────────────────────┤
    │  PORT U  │ PU0  ──► Left IR Emitter           │
    │          │ PU1  ──► Far Left IR Emitter       │
    ├─────────────────────────────────────────────┤
    │  PORT G  │ PG4  ──► Far Right IR Emitter     │
    │          │ PG5  ──► Right IR Emitter          │
    └─────────────────────────────────────────────┘
```

---

## 🔧 Timer Configuration

```
┌──────────────────────────────────────────────────────────────┐
│  T32A0 (Left Motor)    │  T32A3 (Right Motor)                 │
│  ─────────────────────  │  ─────────────────────               │
│  Mode:     16-bit PPG │  Mode:     16-bit PPG                │
│  Prescaler: 1:1       │  Prescaler: 1:1                      │
│  Frequency: 40 kHz     │  Frequency: 40 kHz                   │
│  Period:   2000 counts │  Period:   2000 counts               │
│  Pins:     PA3, PA4    │  Pins:     PC2, PC3                  │
│  Duty:     RG0 = 0-2000│  Duty:     RG0 = 0-2000              │
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

## 🚀 Build & Flash

```bash
# Open in Keil µVision
# Target: TMPM4KNF10AFG
# Debug: CMIS-DAP
# Flash: On-chip 512KB
```

---

## 📝 Module Planning (TODO)

| Module | Status | Description |
|--------|--------|-------------|
| `ctrlTick` | ✅ Done | 1kHz control loop timing |
| `motorCtrl` | 🔄 Planned | PID velocity/position control |
| `encoder` | ✅ Driver done | Hardware quadrature read |
| `irSensor` | 🔄 Planned | Distance measurement + calibration |
| `mazeAlg` | 🔄 Planned | Flood-fill or A* maze solver |
| `motion` | 🔄 Planned | Straight, smooth turn, pivot profiles |
| `wallFollow` | 🔄 Planned | PD wall-following controller |
| `speedRun` | 🔄 Planned | Optimized path execution |

---

## 📚 References

- [TMPM4KNF10AFG Datasheet](https://toshiba.semicon-storage.com/)
- [RM-T32A-C Timer Reference](https://toshiba.semicon-storage.com/info/RM-T32A-C_en_20241129.pdf)
- [RM-A-ENC32-A Encoder Reference](https://toshiba.semicon-storage.com/info/RM-A-ENC32-A_en_20250221.pdf)
- [RM-ADC-I Reference](https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf)
- [RM-DMAC-B DMA Reference](https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf)

---

> **Author:** Kevin Le  
> **Date:** 2026  
> **License:** Proprietary — All rights reserved
