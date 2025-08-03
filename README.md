# SelfPlaying_Piano
ESP32-C3 + PCA9685 64-key Robotic Piano
![5ccb1ed6ac01a7d63d5d7801fed45832](https://github.com/user-attachments/assets/ae4c540f-a16c-498a-9d48-5f8408f17500)

## Intro
 This project is the final project of the 2025 ESAP (Engineering Summer at Penn) Robotics program. It was created by Team Piano 1: Brad Cao, Thomas Wang, and Kelvin Dong. We were instructed by Professors Yim, Pat, and Nick. Special thanks to TAs Zimu Yang and Kayleen Smith for their invaluable support.
 
## Product Demo
Part of Chopin: Fantasy Impromptu


https://github.com/user-attachments/assets/c90b3a9c-bd58-40a1-a728-f804e2af175a


## Features
  - ESP32‑C3 + PCA9685 control for 64 servo actuators (one per piano key)
  - UDP trigger: send a text packet containing the keywords **ex: "S"** to start playback (port **2808**)
  - Event scheduler, tempo scaling, optional velocity (press force), per‑key calibration, rapid repeat/burst tests
  - Bad key masking — skip or remap non‑working keys
  - Multi‑song switching 
  - Optional: latency    using a microphone (not enabled in this code)
  - MIDI range from 32(C2) ~ 96(C7)
  - There are 7 songs configured(shimmy shimmy, BillieJean, ShutupandDance, VivaLaVida, Test, , Fantasy Impromptu)
## Hardware
  - ESP32-C3 dev board x 1
  - PCA9685 servo drive boards x 4
  - MG90s servos x 64
  - External 6v Power Supply x 1
  - Bread Boards and wires
  - Optional: Microphone for latency test

## Program Structure
1. The score is a list of **NoteEvent {startMs, durMs, midi, vel}**
2. The loop checks the current tempo scaled time and, when an event’s startMs is due, it tempo scales durMs and calls **noteOn(midi, now, durScaled)**
4. **noteOn** First remaps badkeys and enforces a recovery time so a key isn`t hitted twice before it gets ready (servo restriction) (for very short notes it applies a minimum hold time)
5. **noteOn** computes a press pulse for that MIDI note (base travel + velocity factor + per-key offset), and sends it to the mapped PCA9685 channel
6. To send a pulse for a given MIDI note, the code maps the note to a **{driverIndex, channel}** pair and outputs setPWM
7. It stores when to release the note (offTime) and a short recovery time for that key.
8. A separate scan later turns notes off exactly at offTime by sending the REST pulse.
  <img width="1316" height="592" alt="image" src="https://github.com/user-attachments/assets/b9694dce-fabc-41f1-9ac8-43dd8d4a1fed" />

## Wiring Map
  ![bab8f6fc1c2f79c1998e6fc46f48c867](https://github.com/user-attachments/assets/8c86c507-1946-4ab2-bfe1-423d8213469c)

## Build & Flash (Arduino IDE)
1. Install **ESP32(by espressif)** , select **ESP32C3** board.  
2. Libraries: **Wire**, **Arduino**, **WiFi**, **WiFiUdp**.  
3. Open **final.ino** and edit:
   - Wi‑Fi: **WIFI_SSID** / **WIFI_PASS** and trigger keyword
   - UDP port: **2808**.
   - **USE_STATIC_IP** is 1 (static) 0 (DHCP)
   - Bad keys list (if used): **BAD_KEYS[]**.
4. Flash and open Serial Monitor at 115200 baud.

## How to control by sending UDP packs
1. Package sender and device connects to your 2.4GHz Wifi and listens on port 2808
2. If the received udp package contains any of the trigger words, it plays the assigned song after 1s of delay
3. It is set as one-shot(first trigger only, skips the song if one has already played) but you can adjust it in the codes
4. Keywords: **song_TRIG** = **S** **B** **D** **V** **T** **91** **69**

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
  9. **X** Reset all servos
  10. **K** Start the key test(full keyboard)
  11. **KX** Stop the key test and exit
  12. **F (midi)** Start a burst on one key（Defaults: interval=120ms repeates 20 times)
  13. **FX** Stop the burst and exit
  14. **O (midi)(delta)** adjust the perkey press offset for one key
  15. **W** Print all non zero perkey calibrations
  16. **R** Clear all perkey calibrations

## Music Score Conversion Rules
**NoteEvent {startMs, durMs, midi, vel}**
1. startMs: absolute start time from song start, in milliseconds
2. durMs: note length in milliseconds
3. midi: MIDI note number (integers)
4. vel: velocity (0–100); kept for future use (the current piano doesn’t support it) 








