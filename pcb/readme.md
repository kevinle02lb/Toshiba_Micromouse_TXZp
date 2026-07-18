<div align="center">

# Micromouse PCB

**Custom 4-Layer Control Board**  
*Integrated MCU, motor drivers, and IR sensing*

<img src="../docs/assets/pcb-3d.png" width="520" alt="PCB 3D render">

</div>

---

## Overview

The mainboard is a custom 4-layer PCB designed in KiCad that carries the full electronics stack on a single micromouse-sized board: the microcontroller, dual motor drivers, four IR wall sensors, and the power and debug interface.

---

## Board Specs

| Spec | Detail |
|------|--------|
| Layers | 4-layer |
| MCU | Toshiba TMPM4KNF10AFG (Cortex-M4F) |
| Motor drivers | 2 × TB67H450AFNG H-bridge |
| Sensing | 4 × IR emitter / receiver pairs |
| Debug | CMSIS-DAP (SWD) via TXB0104 level shifter |
| Supply | 5 V |

---

## Layout

<div align="center">

<img src="../docs/assets/pcb-layout.png" width="480" alt="PCB layout">

*Copper layers and component placement*

</div>

---

## Key Components

| Component | Part | Role |
|-----------|------|------|
| MCU | TMPM4KNF10AFG | Cortex-M4F — control & sensing |
| Motor driver | TB67H450AFNG ×2 | Brushed DC H-bridge, PWM speed control |
| Motors | Pololu #5211 N20 30:1 | Drive, with quadrature encoders |
| IR sensing | IR LED + phototransistor ×4 | Wall detection via ADC |
| Level shifter | TXB0104 | 5 V ↔ 3.3 V for SWD debug |

---

## Schematic

The full schematic is available as a PDF:

[View schematic (PDF)](schematic.pdf)

---

## Files

| File | Contents |
|------|----------|
| `*.kicad_sch` | Schematic source |
| `*.kicad_pcb` | Board layout source |
| `gerbers/` | Fabrication outputs |

---

<div align="center">

[← Back to main README](../README.md)

</div>