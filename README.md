# SelfPlaying_Piano
ESP32-C3 + PCA9685 64-key Robotic Piano
![5ccb1ed6ac01a7d63d5d7801fed45832](https://github.com/user-attachments/assets/ae4c540f-a16c-498a-9d48-5f8408f17500)

## Intro
  This project is the final project of ESAP (Engineering Summer At Penn) 2025 Robotics. It is created by Team Piano 1 (Brad Cao, Thomas Wang, Kelvin Dong). Instructed by Professor Yim, Professor Pat, and Professor Nick. Also thanks alot to the help from TA Zimu Yang and Kayleen Smith.
## Product Demo
Part of Chopin: Fantasy Impromptu


https://github.com/user-attachments/assets/c90b3a9c-bd58-40a1-a728-f804e2af175a


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

## How to control by sending UDP packs
1. Package sender and device connects to your 2.4GHz Wifi and listens on port 2808
2. If the received udp package contains any of the trigger words, it plays the assigned song after 1s of delay
3. It is set as one-shot(first trigger only, skips the song if one has already played) but you can adjust it in the codes

## How to control manually
1. you can play the songs manully by first switching the song by typing  **m (song number-1)** and **S** to start playing
  ### Serial command reference (runtime control)
  Send the commends on the serial monitor
  1. **S** Restarts the current song from beginning immediately
  2. **P** Pause/Reseme
  3. **B (tempo)** set tempoPercent(range from 30%-400%)
  4. **V (velocity)** set velocityPercent(range from 0%-100%)
  5. **L** prints current event index, paused state, tempo, velocity, and time
  6. **T (midi)** Test a single note
  7. **M (song number -1)** Switch to song and play
  8. **MX (song number -1)** Switch to song and pause
  9.  




