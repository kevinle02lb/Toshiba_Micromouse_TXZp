# 🐭 Toshiba Micromouse

> A from-scratch, register-level firmware for an autonomous **IEEE 16×16 micromouse** — a palm-sized robot that maps a maze on its own, finds the center, then races back through the fastest route it found.

<p align="center">
  <img src="docs/assets/robot.jpg" width="520" alt="The micromouse robot">
  <br>
  <em>📷 drop a hero photo of the mouse here</em>
</p>

---

## 🤔 What is this?

Micromouse is a classic robotics challenge: an autonomous robot explores a 16×16 grid maze, discovers the walls, solves for the center, and then does a timed speed run along the best path it found.

This is my build on Toshiba's **TMPM4KNF10AFG** (ARM Cortex-M4). Every peripheral driver is written by hand straight from the Toshiba reference manuals — no vendor HAL — so I actually understand what each register is doing. Motors, quadrature encoders, IR wall sensing, and the maze solver are all coordinated by a single **1 kHz control loop**.

**Highlights**

- 🧠 &nbsp;Flood-fill maze solver (pure logic, no hardware coupling)
- 🎯 &nbsp;1 kHz per-wheel PID speed control
- 🛠️ &nbsp;Fully hand-written HAL, register-level, from the datasheets
- 🔬 &nbsp;Custom 4-layer PCB
- 🧭 &nbsp;Relative-move navigation so turn error never compounds across a run

---

## 🧰 Built With

<p align="center">
  <img src="docs/assets/arm.png"                    height="48" alt="ARM">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/Keil.png"                   height="48" alt="Keil µVision">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/kicad.png"                  height="48" alt="KiCad">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/autodesk-fusion-360_logo_.png" height="48" alt="Fusion 360">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/git.png"                    height="48" alt="Git">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/github.png"                 height="48" alt="GitHub">
  &nbsp;&nbsp;&nbsp;
  <img src="docs/assets/Toshiba-Logo.png"           height="48" alt="Toshiba">
</p>

| Tool | Used for |
|------|----------|
| **Keil µVision** | Firmware IDE, build & flash (CMSIS-DAP) |
| **KiCad** | 4-layer PCB design |
| **Fusion 360** | Chassis & mechanical CAD |
| **C / Cortex-M4** | Register-level embedded firmware |
| **Git** | Version control |

---

## 📸 Gallery

<p align="center">
  <img src="docs/assets/pcb.jpg" width="300" alt="Custom PCB">
  &nbsp;&nbsp;
  <img src="docs/assets/maze.jpg" width="300" alt="Test maze run">
  <br>
  <em>📷 PCB &nbsp;•&nbsp; maze run — swap in your own shots</em>
</p>

---

## 📖 Documentation

The full technical breakdown — architecture, control-loop stages, pin map, timer config, and module status — lives in the source tree:

➡️ **[`src/README.md`](src/README.md)** — architecture & module reference

Other places to look:

- 📂 [`pcb/`](pcb/) — KiCad design files
- 📂 [`src/drivers/`](src/drivers/) — register-level hardware layer
- 📂 [`src/modules/`](src/modules/) — application logic (control, planner, navigation)

---

## 🚀 Quick Start

```text
IDE:     Keil µVision
Target:  TMPM4KNF10AFG
Debug:   CMSIS-DAP
Flash:   On-chip 512 KB
```

---

<p align="center">
  <strong>Kevin Le</strong> &nbsp;•&nbsp; 2026 &nbsp;•&nbsp; 🚧 Work in progress
</p>
