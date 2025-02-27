# DMX-Controlled ESP32 System with Stepper, Servo, Relays, and H-Bridge Motor

This project is an **ESP32-based DMX Slave** that controls:
- **Stepper Motor (TMC2209 with Sensorless Homing)**
- **H-Bridge DC Motor**
- **RC Servos**
- **Relays**
- **DIP Switch for Base DMX Address Selection**

## 🚀 Features
- **DMX Slave** using `esp_dmx`
- **Stepper Motor Control** (via `TMC2209`, with Speed and Position control)
- **Sensorless Homing** using `StallGuard`
- **H-Bridge DC Motor Control** (PWM & Direction)
- **Servo Control** (RC Servo via PWM)
- **Relay Control** (On/Off)
- **DIP Switch-Based DMX Address Selection** (using two 74HC165 shift registers)

---

## 📌 Hardware Setup

### **ESP32 Connections**
| Component         | ESP32 Pin | Description |
|------------------|-----------|-------------|
| **DMX TX**       | `17`       | DMX Transmit |
| **DMX RX**       | `16`       | DMX Receive |
| **DMX Enable**   | `21`       | RS485 Direction Control |
| **Stepper UART** | `14`       | TMC2209 Serial Data |
| **Stepper EN**   | `15`       | Enable (LOW = Active) |
| **Stepper INDEX** | `32`      | Step Pulse Output |
| **Motor A1**     | `4`        | H-Bridge Motor A+ |
| **Motor A2**     | `5`        | H-Bridge Motor A- |
| **Motor B1**     | `18`       | H-Bridge Motor B+ |
| **Motor B2**     | `19`       | H-Bridge Motor B- |
| **Servo 1**      | `2`        | Servo PWM Output |
| **Servo 2**      | `4`        | Servo PWM Output |
| **Relay 1**      | `27`       | Relay Control Output |
| **Relay 2**      | `26`       | Relay Control Output |
| **DIP CLK**      | `18`       | Shift Register Clock |
| **DIP Latch**    | `5`        | Shift Register Latch |
| **DIP Data**     | `34`       | Shift Register Data |

---

## 📡 DMX Channel Mapping

| DMX Address      | Function |
|------------------|----------|
| `BASE + 1`      | **H-Bridge DC Motor Speed** (0-255) |
| `BASE + 2`      | **Servo 1 Angle** (0-255 → 0-180°) |
| `BASE + 3`      | **Servo 2 Angle** (0-255 → 0-180°) |
| `BASE + 4`      | **Stepper Speed** (1-500 Steps/sec) |
| `BASE + 5`      | **Stepper Position** (0 = Home, 1-255 mapped to steps) |
| `BASE + 6`      | **Relay 1 ON/OFF** (127 = ON) |
| `BASE + 7`      | **Relay 2 ON/OFF** (127 = ON) |

> **ℹ️ Base DMX address is set using a DIP switch (74HC165 shift registers).**

---

## ⚙️ Software Setup

### **PlatformIO (Recommended)**
1. Install **[PlatformIO](https://platformio.org/)**
2. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/yourrepo.git
   cd yourrepo
