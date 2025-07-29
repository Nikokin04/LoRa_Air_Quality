# LoRa_Air_Quality
This project implements an outdoor air quality monitoring system based on the STM32F103C8T6  microcontroller, designed to collect and transmit environmental and geolocation data using LoRa communication. It integrates particulate matter and GPS data into Cayenne Low Power Payload (LPP) format for efficient transmission to IoT platforms.

⚙️ Features:
- 🌫 Particulate Matter Monitoring using the PMS5003 sensor (PM1.0 / PM2.5 / PM10)
- 📍 Geolocation support via u-blox NEO-M8N GPS module
- 📡 LoRa communication using RN2903 module
- 🧪 Data formatting with CayenneLPP for optimized IoT compatibility
- 🔁 Automatic reconnection logic to rejoin the LoRa network approximately every 24 hours in case of unexpected disconnection
- 🕒 Watchdog Timer (WDT) enabled on the STM32 for automatic system resets in case of unexpected behavior or hangs
- 🧠 Programmed using Arduino IDE with STM32 core libraries for ease of development and portability

🔧 Hardware:
- Microcontroller: STM32F103C8T6
- Sensors:
  - PMS5003 (Particulate Matter)
  - u-blox NEO-M8N (GPS)
- Communication: RN2903 LoRa module
