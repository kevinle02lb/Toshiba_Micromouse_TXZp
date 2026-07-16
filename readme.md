# Toshiba Micromouse 🐭

> A from-scratch, register-level firmware for an autonomous **IEEE 16×16 micromouse** — a palm-sized robot that maps a maze on its own, finds the center, then races back through the fastest route it found.

<p align="center">
  <img src="docs/assets/robot.jpg" width="520" alt="The micromouse robot">
  <br>
  <em>Hero photo of the mouse goes here</em>
</p>

---

## Overview

Micromouse is a classic robotics challenge: an autonomous robot explores a 16×16 grid maze, discovers the walls, solves for the center, and then runs a timed pass along the best path it found.

This is a build on Toshiba's **TMPM4KNF10AFG** (ARM Cortex-M4). Every peripheral driver is written by hand from the Toshiba reference manuals — no vendor HAL — so each register access is understood rather than abstracted away. Motors, quadrature encoders, IR wall sensing, and the maze solver are all coordinated by a single **1 kHz control loop**.

**Highlights**

- Flood-fill maze solver — pure logic, no hardware coupling
- 1 kHz per-wheel PID speed control
- Fully hand-written HAL, register-level, from the datasheets
- Custom 4-layer PCB
- Relative-move navigation, so turn error never compounds across a run

---

## Built With

<p align="center">
  <img src="docs/assets/arm.png"                       height="48" alt="ARM">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/Keil.png"                      height="48" alt="Keil µVision">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/kicad.png"                     height="48" alt="KiCad">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/autodesk-fusion-360_logo_.png" height="48" alt="Fusion 360">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/git.png"                       height="48" alt="Git">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/Toshiba-Logo.png"              height="48" alt="Toshiba">
</p>

| Tool | Used for |
|------|----------|
| **Keil µVision** | Firmware IDE, build & flash (CMSIS-DAP) |
| **KiCad** | 4-layer PCB design |
| **Fusion 360** | Chassis & mechanical CAD |
| **C / Cortex-M4** | Register-level embedded firmware |
| **Git** | Version control |

---

## Gallery

<p align="center">
  <img src="docs/assets/pcb.jpg" width="300" alt="Custom PCB">
  &nbsp;&nbsp;
  <img src="docs/assets/maze.jpg" width="300" alt="Test maze run">
  <br>
  <em>PCB and maze run — replace with your own images</em>
</p>

---

## Documentation

The full technical breakdown — architecture, control-loop stages, pin map, timer configuration, and module status — lives in the source tree:

**[`src/README.md`](src/README.md)** — architecture & module reference

Other locations:

- [`pcb/`](pcb/) — KiCad design files
- [`src/drivers/`](src/drivers/) — register-level hardware layer
- [`src/modules/`](src/modules/) — application logic (control, planner, navigation)

---

## Quick Start

```text
IDE:     Keil µVision
Target:  TMPM4KNF10AFG
Debug:   CMSIS-DAP
Flash:   On-chip 512 KB
```

---

<p align="center">
  <strong>Kevin Le</strong> &nbsp;•&nbsp; 2026 &nbsp;•&nbsp; Work in progress
</p>