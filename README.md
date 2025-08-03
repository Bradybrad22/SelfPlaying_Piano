# SelfPlaying_Piano
ESP32-C3 + PCA9685 64-key Robotic Piano
## Intro
  This project is the final project of ESAP (Engineering Summer At Penn) 2025 Robotics. It is created by Team Piano 1 (Brad Cao, Thomas Wang, Kelvin Dong). Instructed by Professor Yim, Professor Pat, and Professor Nick. Also thanks alot to the help from TA Zimu Yang and Kayleen Smith.
## Product Demo

## Features
  - ESP32‑C3 + PCA9685 control for 64 servo actuators (one per piano key).
  - UDP trigger: send a text packet containing the keywords **ex: "S"** to start playback (port **2808**).  
  - Event scheduler, tempo scaling, optional velocity (press force), per‑key calibration, rapid repeat/burst tests.
  - Bad key masking — skip or remap non‑working keys.
  - Multi‑song switching hooks .
  - Optional: latency    using a microphone (not enabled in this code by default).
  - MIDI range from 32(C2) ~ 96(C7
## Hardware
  - ESP32-C3 dev board x 1
  - PCA9685 servo drive boards x 4
  - MG90s servos x 64
  - External 6v Power Supply x 1
  - Bread Boards and wires
  - Optional: Microphone for latency test
  
## Wiring Map
  ![bab8f6fc1c2f79c1998e6fc46f48c867](https://github.com/user-attachments/assets/8c86c507-1946-4ab2-bfe1-423d8213469c)

## Build & Flash (Arduino IDE)
1. Install **ESP32(by espressif)** , select **ESP32C3** board.  
2. Libraries: `Wire`, `Arduino`, `WiFi`, `WiFiUdp`.  
3. Open `final.ino` and edit:
   - Wi‑Fi: `WIFI_SSID` / `WIFI_PASS` and trigger keyword
   - UDP port: `2808`.
   - Bad keys list (if used): `BAD_KEYS[]`.
4. Flash and open Serial Monitor at 115200 baud.
5. you can play the songs manully by first switching the song by typing  **m <song number-1>** and **S** to start playing


