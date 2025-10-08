# 🌐 ESP32 IoT Control System with Tuya App

An Internet of Things (IoT) project built using **ESP32**, **Tuya Smart App**, and **Arduino**, featuring real-time **temperature and pressure monitoring**, **relay control**, and **OLED-based status display**.  
The system can be controlled both **remotely via Tuya Cloud** and **manually through physical buttons**.

> 🧩 The **Wokwi simulation** is included only for **pinout and circuit visualization** — Tuya Cloud control and data communication work on **real ESP32 hardware**.

---

## 🔗 Simulation for pinout and circuit visualization

[👉 View Project on Wokwi](https://wokwi.com/projects/444187946000695297)

---

## 🧠 Features

- 🌡️ **Temperature & Pressure Monitoring** using the BMP280 sensor  
- ⚙️ **3 Relays** controllable via Tuya Smart App or physical buttons  
- 🖥️ **0.96" OLED Display** showing live temperature, pressure, and relay states  
- 🚨 **Manual Fault Mode** toggle via Button1 for testing system behavior  
- ☁️ **Tuya Cloud Integration** for IoT connectivity and remote control  
- 🔄 **Real-time Synchronization** between local buttons and mobile app controls  

---

## ⚙️ Hardware Components

| Component | Description |
|------------|-------------|
| **ESP32** | Main microcontroller with built-in Wi-Fi |
| **BMP280 Sensor** | Measures temperature and atmospheric pressure |
| **OLED Display (SSD1306)** | Displays real-time readings and relay states |
| **3x Relays** | Control external devices or appliances |
| **Push Buttons** | Local control and fault simulation input |

---

## 🧰 Software & Tools

- **Arduino IDE**  
- **Tuya IoT Platform** (for device registration and cloud control)  
- **Wokwi Simulator** (for pinout and circuit visualization)  
- **Adafruit Libraries:**  
  - `Adafruit_SSD1306`  
  - `Adafruit_GFX`  
  - `Adafruit_BMP280`  
- **WiFiManager** (for wireless network configuration)  

---

## 📂 Project Structure

```
esp32-tuya-iot-system/
├── src/
│   └── main.ino
├── README.md
└── assets/
    └── circuit_diagram.png
```

---

## 📱 Tuya Integration

1. Register or log in to [Tuya IoT Platform](https://iot.tuya.com/)  
2. Create a new **Device Project** for ESP32  
3. Configure **Data Points (DPs)** for relays and sensors  
4. Flash the ESP32 with your Arduino sketch  
5. Pair the device using the **Tuya Smart App**  
6. Monitor **temperature**, **pressure**, and control **relays** from your phone anywhere in the world 🌍  

---

## 🧩 Circuit Overview

The **Wokwi simulation** provides a clear reference for wiring the ESP32 with the BMP280 sensor, OLED display, and relays.  
It does **not** simulate Tuya cloud behavior but helps visualize component connections and pinouts.

**Key Connections:**
- BMP280 → I2C (SDA, SCL)  
- OLED (SSD1306) → I2C (same lines)  
- Relays → GPIO pins (defined in `main.ino`)  
- Buttons → Digital inputs with pull-down resistors  

---

## 🪄 Future Improvements

- 📈 Add MQTT data logging for temperature and pressure trends  
- 🖥️ Develop a local **web dashboard** for offline control and monitoring  
- ⚡ Add **energy consumption tracking** for connected loads  
- 🔔 Push notifications for threshold-based alerts (e.g., high temperature)  

---

## 👤 Authors



**Hasan Bilal**  
🔧 Embedded Systems Developer  
📍 Islamabad, Pakistan  

**Muhammad Jahan Zaib**  
🎓 Embedded Systems & IoT Enthusiast  
📍 Islamabad, Pakistan  
📎 [LinkedIn Profile](https://www.linkedin.com/in/muhammad-jahan-zaib-b17a99273/)  

⭐ If you found this project helpful, consider giving it a **star** on GitHub!

---

## 🏷️ GitHub Topics

`esp32` • `tuya` • `iot` • `arduino` • `bmp280` • `oled-display` • `embedded-systems` • `smart-home` • `wifi` • `sensor-network`
