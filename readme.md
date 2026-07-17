<div align="center">

# 🐭 Toshiba Micromouse

**Autonomous 16×16 Maze-Solving Robot**  
*Register-level firmware on ARM Cortex-M4F*

<img src="docs/assets/robot.jpg" width="520" alt="The micromouse robot">

</div>

---

## Overview

Micromouse is a classic robotics competition: an autonomous palm-sized robot explores a 16×16 grid maze, maps the walls, solves for the center, and then races back through the fastest route it discovered.

This project implements the entire stack from scratch — custom 4-layer PCB, mechanical chassis, and register-level firmware — with no reliance on external HAL libraries.

---

## Platform

<div align="center">

<img src="docs/assets/Toshiba-Logo.png" height="60" alt="Toshiba">

### **Toshiba TMPM4KNF10AFG** — ARM Cortex-M4F

</div>

The TMPM4KNF10AFG is an industrial-grade ARM Cortex-M4F microcontroller from Toshiba's M4K motor-control family. Engineered for **real-time motion systems**, it is deployed across industrial and consumer applications including BLDC/PMSM motor drives, HVAC compressors, power tools, pumps, fans, and factory automation equipment.

**Key specs leveraged in this project:**

| Feature | Application |
|---------|-------------|
| Cortex-M4F + FPU | Real-time maze solving & control algorithms |
| Hardware QEI (A-ENC32) | Wheel encoder feedback |
| Advanced Motor Timers | Complementary PWM with dead-time for H-bridge drive |
| Trigger-synced ADC | IR wall-sensing with hardware timing |

The same silicon powering industrial three-phase motor drives runs a 1 kHz closed-loop PID over two brushed DC gear-motors, with dedicated hardware handling encoder counting and motor PWM so the CPU stays free for navigation and flood-fill maze solving.

---

## Built With

<div align="center">

| <img src="docs/assets/arm.png" height="40" alt="ARM"> | <img src="docs/assets/Keil.png" height="40" alt="Keil µVision"> | <img src="docs/assets/kicad.png" height="40" alt="KiCad"> | <img src="docs/assets/autodesk-fusion-360_logo_.png" height="40" alt="Fusion 360"> | <img src="docs/assets/git.png" height="40" alt="Git"> |
|:---:|:---:|:---:|:---:|:---:|
| **ARM Cortex-M4** | **Keil µVision** | **KiCad** | **Fusion 360** | **Git** |
| Register-level C | IDE / Build / Flash | 4-layer PCB | Chassis CAD | Version Control |

</div>

---

## Architecture Highlights

- **Flood-fill maze solver** — pure logic, zero hardware coupling
- **1 kHz per-wheel PID** speed control
- **Fully hand-written HAL** from datasheets, no external libraries
- **Relative-move navigation** — turn error never compounds across a run
- **Custom 4-layer PCB** with integrated motor drivers & IR sensors

---

## Gallery

<div align="center">

<img src="docs/assets/pcb.jpg" width="300" alt="Custom PCB"> &nbsp;&nbsp; <img src="docs/assets/maze.jpg" width="300" alt="Test maze run">

*Custom PCB and test maze run*

<br>

<img src="docs/assets/fusion_robot.png" width="480" alt="Fusion 360 Robot Render">

*Fusion 360 render of the full mechanical assembly*

</div>

---

## Documentation

| Path | Contents |
|------|----------|
| [`src/README.md`](src/README.md) | Firmware architecture, control loops, pin map, timer config |
| [`pcb/`](pcb/) | KiCad design files & fabrication outputs |
| [`src/drivers/`](src/drivers/) | Register-level hardware abstraction layer |
| [`src/modules/`](src/modules/) | Application logic: planner, navigator, controller |

---

## Quick Start

```text
IDE:     Keil µVision
Target:  TMPM4KNF10AFG (ARM Cortex-M4F)
Debug:   CMSIS-DAP
Flash:   512 KB on-chip
```

---

<div align="center">

**Kevin Le** &nbsp;•&nbsp; 2026 &nbsp;•&nbsp; Work in Progress

</div>
