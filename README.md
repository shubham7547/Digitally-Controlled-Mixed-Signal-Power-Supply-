# Digitally Controlled Mixed-Signal Power Supply

[![Microcontroller](https://img.shields.io/badge/MCU-Arduino%20Nano%20Every-blue.svg)](https://www.arduino.cc/)
[![DAC](https://img.shields.io/badge/DAC-TLC5618A%20(12--bit%20SPI)-orange.svg)]()
[![Display](https://img.shields.io/badge/Display-ILI9341%20320x240%20TFT-green.svg)]()
[![EDA](https://img.shields.io/badge/EDA-KiCad-blueviolet.svg)](https://www.kicad.org/)
[![Date](https://img.shields.io/badge/Date-March%202026-lightgrey.svg)]()

A high-performance, digitally controlled benchtop power supply that bridges digital precision with an ultra-fast analog regulation loop. The system delivers an adjustable **0 to 30.0 V** output at up to **1.0 A**, featuring dual rotary quadrature encoders with digit-level selection, high-resolution readbacks on a 2.8" color TFT display, and a non-blocking embedded firmware architecture.

Developed at the **Centre for Electronic Design and Technology (CEDT)**, **Netaji Subhas University of Technology (NSUT), New Delhi**.

---

## Table of Contents

- [Architectural Philosophy](#architectural-philosophy)
- [System Architecture](#system-architecture)
- [Key Specifications](#key-specifications)
- [Hardware & Circuit Design](#hardware--circuit-design)
  - [1. AC Input & Bulk Rectification](#1-ac-input--bulk-rectification)
  - [2. Dual Auxiliary Buck Regulators](#2-dual-auxiliary-buck-regulators)
  - [3. Digital Setpoint Generation](#3-digital-setpoint-generation)
  - [4. Analog Voltage Regulation Loop](#4-analog-voltage-regulation-loop)
  - [5. Current Sensing & Differential Amplification](#5-current-sensing--differential-amplification)
- [Pin Assignment Table](#pin-assignment-table)
- [Firmware Architecture](#firmware-architecture)
  - [Interrupt-Driven Quadrature Decoding](#interrupt-driven-quadrature-decoding)
  - [Non-Blocking Scheduling](#non-blocking-scheduling)
  - [Software Current Limit Foldback](#software-current-limit-foldback)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
  - [Required Libraries](#required-libraries)
  - [Hardware Setup & Flashing](#hardware-setup--flashing)
- [Author & Affiliation](#author--affiliation)
- [License](#license)

---

## Architectural Philosophy

Simpler DIY microcontroller-based power supplies often place the digital microcontroller inside the primary feedback loop or use PWM/software loops to regulate output voltage, leading to slow transient response, ADC latency, and stability issues.

This design enforces a strict separation of concerns:
1. **Digital Setpoint Domain:** The **Arduino Nano Every** and **TLC5618A 12-bit DAC** determine what target voltage/current limit is desired. They never carry load current and never sit directly inside the fast closed loop.
2. **Analog Control Domain:** An **LM358 op-amp error amplifier** continuously compares the scaled output feedback against the DAC setpoint in real time, driving high-power series-pass transistors (**MJE2955T PNP**) instantaneously without digital latency.

---

## System Architecture

```mermaid
flowchart TD
    Mains["Mains 230V AC"] --> T1["Step-Down Transformer\n(Off-Board)"]
    T1 --> Bridge["Bridge Rectifier (1N4007)\n+ 2x 4700µF Series Caps (50V eff.)"]
    Bridge --> RawDC["Raw DC Bus (~36V Peak)"]

    RawDC --> VR6["LM2575 Buck 1\nStep-down to 5.29V"]
    VR6 --> MCU["Arduino Nano Every\n(5V Digital Rail)"]

    RawDC --> VR5["LM2575 Buck 2\nStep-down to 30.0V Rail"]
    VR5 --> OpAmp["LM358 Error Amplifier\n(Analog Rail)"]

    MCU -->|Hardware SPI| DAC["TLC5618A 12-Bit DAC\n(Vref Generation)"]
    DAC -->|Vref Setpoint| OpAmp

    RawDC --> PassTrans["MJE2955T Power Transistor\nSeries-Pass Output Stage"]
    OpAmp -->|Drive Base| PassTrans

    PassTrans --> Load["Output Terminal (0–30V, 0–1A)"]
    Load --> Shunt["Low-Side Current Shunt (0.3 Ω)"]

    Load -->|Voltage Divider (R4/R5)| MCU_ADC_V["ADC Pin A1 (V_sense)"]
    Load -->|Voltage Divider| OpAmp
    Shunt -->|Diff-Amp (LM358)| MCU_ADC_I["ADC Pin A0 (I_sense)"]

    MCU -->|Hardware SPI| TFT["ILI9341 320x240 Color TFT\nSet V/I, Out V/I, CV/CC"]
    Encoders["Dual Rotary Encoders\n+ Push Switches"] -->|Interrupts D2, D6| MCU
```

---

## Key Specifications

| Parameter | Specification | Notes |
| :--- | :--- | :--- |
| **Output Voltage Range** | $0.000\text{ V}$ to $30.000\text{ V}$ | Continuously adjustable |
| **Output Current Range** | $0.000\text{ A}$ to $1.000\text{ A}$ | Software-limited foldback |
| **Microcontroller** | Arduino Nano Every | ATmega4809 @ 16 MHz |
| **DAC Resolution** | 12-bit ($4096\text{ steps}$) | TLC5618A Dual SPI DAC |
| **DAC Reference Voltage** | $1.090\text{ V}$ | Shared with internal ADC reference |
| **Voltage Control Gain** | $22.886\text{ V/V}$ | $V_{\text{out}} / V_{\text{dac}}$ |
| **Raw DC Bus Voltage** | $\approx 36\text{ V peak}$ | Rectified AC secondary |
| **Digital Auxiliary Rail** | $5.29\text{ V}$ | Regulated via LM2575-ADJ (VR6) |
| **Analog Auxiliary Rail** | $30.02\text{ V}$ | Regulated via LM2575-ADJ (VR5) |
| **Current Sense Shunt** | $0.3\ \Omega$ | Low-side shunt resistor |
| **Current Sense Gain** | $0.7398\text{ V/A}$ | $0.216 \times 3.425$ diff-amp gain |
| **Display** | 2.8" ILI9341 Color TFT | $320 \times 240$ resolution via SPI |
| **Voltage Digit Steps** | $1\text{ mV},\ 10\text{ mV},\ 100\text{ mV},\ 1\text{ V},\ 10\text{ V}$ | 5 selectable cursor positions |
| **Current Digit Steps** | $1\text{ mA},\ 10\text{ mA},\ 100\text{ mA},\ 1\text{ A}$ | 4 selectable cursor positions |

---

## Hardware & Circuit Design

### 1. AC Input & Bulk Rectification
* Off-board mains transformer secondary connects via a 2-pin heavy-duty terminal block.
* Rectification via a full-wave 1N4007 diode bridge produces a $\approx 36\text{ V}$ peak raw DC bus.
* Bulk filtering uses **two $4700\ \mu\text{F},\ 25\text{ V}$ electrolytic capacitors in series** ($C_2, C_4$) to double the effective breakdown voltage rating to $50\text{ V}$, well above the $36\text{ V}$ peak headroom.

### 2. Dual Auxiliary Buck Regulators
To supply both digital control logic and high-voltage analog regulation without wasting excessive power through linear dropouts, two independent LM2575 switching buck converters are employed:
* **VR6 (Digital 5.29V Rail):** $R_2 = 3.3\text{ k}\Omega,\ R_1 = 1\text{ k}\Omega \implies V_{\text{out}} = 1.23 \times (1 + 3300/1000) = 5.29\text{ V}$. Powers the Arduino 5V net and logic.
* **VR5 (Analog 30.0V Rail):** $R_2 = 110\text{ k}\Omega,\ R_1 = 4.7\text{ k}\Omega \implies V_{\text{out}} = 1.23 \times (1 + 110000/4700) = 30.02\text{ V}$. Powers the LM358 op-amp error amplifier stage.

### 3. Digital Setpoint Generation
* Uses a **TLC5618A** 12-bit dual-channel DAC over hardware SPI.
* Output Channel A generates the analog reference voltage $V_{\text{ref}}$, filtered by capacitor $C_1$.
* Calculated DAC code:
  $$\text{Code} = \left(\frac{V_{\text{target}} / 22.886}{2 \times 1.090}\right) \times 4096$$

### 4. Analog Voltage Regulation Loop
* Implemented with an **LM358** operational amplifier configured as a fast error amplifier:
  - Non-inverting input ($+$): $V_{\text{ref}}$ from the TLC5618A DAC.
  - Inverting input ($-$): $V_{\text{sense}}$ from the output voltage divider.
* Op-amp output directly drives the base of the series-pass power transistor (**MJE2955T PNP**). Any instantaneous load variation is corrected within microseconds by the analog loop without waiting for microcontroller polling.

### 5. Current Sensing & Differential Amplification
* Output current flows through a precision $0.3\ \Omega$ low-side shunt ($R_3$).
* The differential voltage is amplified by the second half of the LM358 package ($0.7398\text{ V/A}$) and routed directly to ADC channel `A0` for continuous readback and software foldback protection.

---

## Pin Assignment Table

| Arduino Nano Every Pin | Signal Name | Connected To | Function / Description |
| :--- | :--- | :--- | :--- |
| **D2** | `ENC_V_A` | Voltage Encoder Phase A | Hardware interrupt, quadrature phase A |
| **D3** | `ENC_V_B` | Voltage Encoder Phase B | Quadrature phase B input |
| **D4** | `ENC_V_SW` | Voltage Encoder Push Switch | Switch to cycle active voltage edit digit |
| **D5** | `ENC_I_SW` | Current Encoder Push Switch | Switch to cycle active current limit digit |
| **D6** | `ENC_I_A` | Current Encoder Phase A | Hardware interrupt, quadrature phase A |
| **D7** | `ENC_I_B` | Current Encoder Phase B | Quadrature phase B input |
| **D8** | `TFT_CD` | ILI9341 D/C Pin | Display Data / Command selection |
| **D9** | `TFT_CS` | ILI9341 CS Pin | Display Chip Select |
| **D10** | `DAC_CS` | TLC5618A ~CS Pin | DAC Chip Select |
| **D11** | `MOSI` | Shared SPI Bus | Hardware SPI Data Out (TFT + DAC) |
| **D12** | `MISO` | Shared SPI Bus | Hardware SPI Data In (unused) |
| **D13** | `SCK` | Shared SPI Bus | Hardware SPI Clock (TFT + DAC) |
| **A0** | `ADC_ISENSE` | Current-Sense Diff-Amp Out | Analog input, output current readback |
| **A1** | `ADC_VSENSE` | Output Voltage Divider Out | Analog input, output voltage readback |
| **AREF** | `Vref` | Shared Reference Net | Common 1.090V DAC reference / ADC ref |

---

## Firmware Architecture

The firmware (`adj_power_combined.ino`) implements a fully non-blocking architecture without any `delay()` calls:

### Interrupt-Driven Quadrature Decoding
* Both rotary encoders trigger pin-change interrupts on their Phase A signals (`D2` and `D6`).
* A 2-bit quadrature Gray-code state machine evaluates transition validity to reject switch bounce and deliver crisp rotation response.

### Non-Blocking Scheduling
* **Encoder & DAC update:** Handled immediately on knob turn or switch press ($200\text{ ms}$ debounce).
* **Sensor Acquisition Loop:** Runs on a dedicated $100\text{ ms}$ periodic cadence (`SENSE_INTERVAL_MS`).
* **TFT Refresh Loop:** Runs on a distinct $150\text{ ms}$ cadence (`DISPLAY_INTERVAL_MS`), rewriting only modified text fields to prevent screen flicker and bus contention.

### Software Current Limit Foldback
If the measured load current exceeds the user-configured current limit ($I_{\text{out}} > I_{\text{limit}}$):
1. The firmware flags `ccActive = true` and updates the UI status badge from **CV** (Green) to **CC** (Yellow).
2. The firmware automatically walks the DAC voltage setpoint downward in small incremental steps until load current falls back within bounds.
3. Once the overload is removed, the original voltage setpoint is automatically restored.

---

## Repository Structure

```
Benchtop-Power-Supply/
├── README.md                                    # Comprehensive project documentation
├── .gitignore                                   # KiCad temporary & backup ignore rules
├── Block Diagram/
│   └── block_diagram.png                        # High-level system architecture block diagram
├── Code Files/
│   └── adj_power_combined/
│       └── adj_power_combined.ino               # Non-blocking firmware for Arduino Nano Every
├── Datasheet/
│   ├── LM2575.PDF                               # Step-down switching regulator datasheet
│   ├── lm358.pdf                                # Dual operational amplifier datasheet
│   ├── MJE2955T.PDF                             # PNP silicon power transistor datasheet
│   └── TLC5618.PDF                              # Dual 12-bit digital-to-analog converter datasheet
├── Kicad Files/
│   └── Adj_power/
│       ├── Adj_power.kicad_sch                  # KiCad schematic design
│       ├── Adj_power.kicad_pcb                  # KiCad printed circuit board layout
│       ├── Adj_power.kicad_pro                  # KiCad project file
│       ├── Adj_power.pdf                        # Printable schematic diagram PDF
│       └── bom/
│           └── ibom.html                        # Interactive HTML Bill of Materials
├── Report/
│   └── pwr_supply.pdf                           # Full technical engineering report document
└── Report Source File/
    └── pwr_supply/
        ├── adj_power_combined.ino               # Firmware source copy
        ├── block_diagram.png                    # Block diagram graphic
        ├── cedtnew.png                          # Institutional logo
        └── schematic.jpg                        # Circuit schematic render
```

---

## Getting Started

### Required Libraries
Install the following libraries via the Arduino Library Manager or place them in your `libraries/` directory:
- **SPI** (Standard Arduino SPI library)
- **LCDWIKI_GUI** (Graphical primitives)
- **LCDWIKI_SPI** (Hardware SPI driver for ILI9341)

### Hardware Setup & Flashing
1. Connect your **Arduino Nano Every** to your PC using a micro-USB cable.
2. Open `Code Files/adj_power_combined/adj_power_combined.ino` in Arduino IDE.
3. Configure your IDE settings under **Tools**:
   - **Board:** Arduino Nano Every (or megaAVR Boards)
   - **Registers emulation:** None (ATMEGA4809 native)
   - **Port:** Select the corresponding COM port.
4. Verify and **Upload** the firmware sketch.
5. The ILI9341 display will illuminate and render the default UI with $0.000\text{ V}$ setpoint and $1.000\text{ A}$ current limit in **CV** mode.

---

## Author & Affiliation

* **Shubham Kumar**

**Centre for Electronic Design and Technology (CEDT)**  
**Netaji Subhas University of Technology (NSUT), New Delhi**  
*Project Date: March 2026*

---

## License

This project is open-source hardware and software available under the [MIT License](LICENSE).
