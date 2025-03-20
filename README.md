# DMX-Controlled ESP32 System with Stepper, Servo, Relays, and H-Bridge Motor

This project is an **ESP32-based DMX Slave** that controls:
- **Stepper Motor (TMC2209 with Sensorless Homing)**
- **2 x H-Bridge DC Motor**
- **4 x RC Servos**
- **2 x Relays**
- **DIP Switch for Base DMX Address Selection**

## 🚀 Features
- **DMX Slave** using `esp_dmx`
- **Stepper Motor Control** (via `TMC2209`, with Speed and Position control)
- **Sensorless Homing** using `StallGuard`
- **H-Bridge DC Motor Control** (PWM & Direction)
- **Servo Control** (RC Servo via PWM)
- **Relay Control** (On/Off)
- **DIP Switch-Based DMX Address Selection**

---
# MultiDMX Web Configuration Interface

## Overview
The **MultiDMX Web Configuration Interface** allows users to configure stepper motors, servos, and DMX settings via a web-based UI served by an ESP32 microcontroller. Settings are saved persistently and can be adjusted dynamically.

## Features
✅ Web-based UI for easy configuration  
✅ Saves settings in **ESP32 Preferences** (persistent storage)  
✅ Supports **stepper motors, servos, and DMX controls**  
✅ Built-in **Wi-Fi access point** for standalone configuration  
✅ **Dynamic settings loading** via JavaScript (AJAX)  

---

## Getting Started

### 1. Powering On & Connecting to Wi-Fi
1. Power on the ESP32 module.
2. The ESP32 creates a **Wi-Fi Access Point** named:  
   **`MultiDMX-XXXX`** (where `XXXX` is the last 4 bytes of the ESP32 MAC address).
3. Connect to this Wi-Fi network
4. Open a web browser and go to:  
   **`http://192.168.4.1/`**  

---

## Web Interface Overview

### Configuration Page (`/`)
- Displays a **static HTML page**.
- Loads current configuration dynamically using JavaScript from **`/config`** API.
- Allows modifying stepper, servo, and DMX settings.

### JSON Configuration API (`/config`)
- Returns **current settings in JSON format** (used by the webpage to auto-fill form fields).

### Save Settings (`/save`)
- The form submits changes to **`/save`**.
- Settings are stored in ESP32 **Preferences**.
- ESP32 **automatically restarts** if changes are detected.

### Reset to Defaults (`/reset`)
- Clicking "Reset to Defaults" clears all settings and restarts the ESP32.

---

## Configuration Options

### Stepper Motor Settings
| Setting | Description |
|---------|-------------|
| **Stepper Current** | Adjusts the stepper driver current |
| **Stepper Scaling Factor** | Scale for step position |
| **Stepper Max Speed** | Maximum step speed (steps/sec) |
| **Stepper Acceleration** | Acceleration (steps/sec²) |
| **Stepper Homing Speed** | Speed during homing sequence |
| **Stepper Homing Acceleration** | Acceleration during homing |
| **Stepper Homing Timeout** | Timeout (ms) for homing |
| **Stepper Homing Step Limit** | Maximum steps for homing |
| **Stepper Reversed** | Checkbox to reverse stepper direction |

### Servo Settings (1-4)
| Setting | Description |
|---------|-------------|
| **Servo Min Micros** | Minimum PWM pulse width |
| **Servo Max Micros** | Maximum PWM pulse width |
| **Servo Reversed** | Checkbox to reverse servo direction |

---

## DMX Configuration & Ports

The following **DMX channels** are used in the system:

| DMX Channel | Function |
|-------------|----------|
| **Base DMX Address + 0*** | Controls Servo 1 Angle, mapped 0-255 → 0-180° |
| **Base DMX Address + 1*** | Controls Servo 2 Angle, mapped 0-255 → 0-180° |
| **Base DMX Address + 2*** | Controls Servo 3 Angle, mapped 0-255 → 0-180° |
| **Base DMX Address + 3*** | Controls Servo 4 Angle, mapped 0-255 → 0-180° |
| **Base DMX Address + 4*** | Controls Motor A Speed & Direction, 1-127 reverse, 129-255 forward, 0/128: standstill  |
| **Base DMX Address + 5*** | Controls Motor B Speed & Direction, 1-127 reverse, 129-255 forward, 0/128: standstill  |
| **Base DMX Address + 6*** | Sets stepper speed, mapped 0-255 → 1-"Stepper Max Speed" |
| **Base DMX Address + 7*** | Sets stepper position , mapped 0-255 → 1-"Stepper Scaling Factor" |
| **Base DMX Address + 8*** | Controls Relay 1 0-127 off, 128-255 on |
| **Base DMX Address + 9*** | Controls Relay 2 0-127 off, 128-255 on |

- The **Base DMX Address** is determined via **DIP switch input**.

---

## How Settings Work
1. Open **`http://192.168.4.1/`** in your browser.
2. The webpage **fetches current settings** from the ESP32 using **`/config`**.
3. Adjust values in the form.
4. Click **"Save Settings"** – the ESP32 will **restart** if settings changed.

---

## Debugging & Serial Output

### Viewing Logs
- Use **Arduino Serial Monitor** at `115200` baud rate.
- The ESP32 prints **Wi-Fi connection logs, configuration updates, and DMX data**.

### Resetting the ESP32
- To clear saved settings, use the **"Reset to Defaults"** button on the web interface.



