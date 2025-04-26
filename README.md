# SubZero - Underwater-Drone
# Underwater RCV Controller 🤿⚙️

This is the control system for my custom-designed Underwater Remotely Controlled Vehicle (RCV). It features real-time depth and heading stabilization using dual PID loops.

## 🚀 Features

- 🎯 **Dual PID Control** – for accurate depth and heading stabilization.
- 🔧 **Real-time Serial Tuning** – adjust PID values live while testing.
- 💡 **Thruster Control** – precise response based on sensor input.

## 🛠️ Hardware Setup

- Microcontroller: `ESP32 / Arduino-compatible`
- Thrusters: `Brushed or brushless with ESCs`
- Sensors: `IMU + Depth sensor (e.g. barometric or ultrasonic)`

## 📂 File Overview

| File | Description |
|------|-------------|
| `main.ino` | Main code running the PID loops and thruster logic |
| `pid.h` / `pid.cpp` | Modular PID class for clean integration |
| `serial_tuner.h` | Live tuning interface via serial monitor |

## 📷 Preview

> Add a photo or CAD render of your underwater RCV here!

## 🧪 How to Use

1. Upload the code to your board.
2. Connect sensors and thrusters.
3. Use the serial monitor to tune your PID values.
4. Test in a controlled water environment 🧪🌊

## 📜 License

This project is open source under the MIT License.

---
