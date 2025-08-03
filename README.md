# SelfPlaying_Piano
ESP32-C3 + PCA9685 64-key Robotic Piano
## Intro
  This project is the final project of ESAP (Engineering Summer At Penn) 2025 Robotics. It is created by Team Piano 1 (Brad Cao, Thomas Wang, Kelvin Dong). Instructed by Professor Yim, Professor Pat, and Professor Nick. Also thanks alot to the help from TA Zimu Yang and Kayleen Smith.
## Features
  - ESP32‑C3 + PCA968* control for 64 servo actuators (one per piano key).
  - UDP trigger: send a text packet containing the keywords **ex: "S"** to start playback (port **2808**).  
  - Event scheduler, tempo scaling, optional velocity (press force), per‑key calibration, rapid repeat/burst tests.
  - Bad key masking — skip or remap non‑working keys.
  - Multi‑song switching hooks .
  - Optional: latency measurement using a microphone (not enabled in this code by default).
  - MIDI range from 32(C2) ~ 96(C7
