# DrawBot — Finger-Drawn Path Robot

> **Draw a path in the air with your hand → the robot drives that exact path on the floor.**

DrawBot is a two-part system: a Python app running on your laptop handles the webcam, hand tracking, path planning, and motor math, while a microcontroller (ESP8266 or ESP32) on the robot receives the commands over Wi-Fi and replays them to drive the motors.

---

## Table of Contents

- [Absolute Beginner Guide (Start Here!)](#absolute-beginner-guide-start-here)
- [Project Files](#project-files)
- [Hardware Requirements](#hardware-requirements)
- [Wiring Diagrams (ESP8266 & ESP32)](#wiring-diagrams)
- [Software Requirements](#software-requirements)
- [How to Use](#how-to-use)
- [Command Reference](#command-reference)
- [Keyboard Shortcuts](#keyboard-shortcuts)
- [Calibration Guide](#calibration-guide)
- [Tuner GUI](#tuner-gui)
- [Parameter Reference](#parameter-reference)
- [Troubleshooting](#troubleshooting)
- [Wi-Fi and Protocol Reference](#wi-fi--protocol-quick-reference)

---

## Absolute Beginner Guide (Start Here!)

If you received this folder as a `.zip` file and have never coded before, follow these exact steps to get everything running.

### 1. Extract the Zip File
Right-click the zip file and click **Extract All...**. Extract it somewhere easy to find, like your `Documents` or `Desktop`. *Do not run the code from inside the zip file directly!*

### 2. Install VS Code
1. Go to [code.visualstudio.com](https://code.visualstudio.com/) and download Visual Studio Code.
2. Install it with all the default options.

### 3. Install Python
1. Go to [python.org/downloads](https://www.python.org/downloads/) and download the latest Python installer for Windows.
2. **CRITICAL STEP**: When you open the installer, look at the very bottom of the first screen. You MUST check the box that says **"Add Python to PATH"** before clicking Install. If you miss this, nothing will work!
3. Click "Install Now".

### 4. Open the Project in VS Code
1. Open VS Code.
2. Click **File > Open Folder**.
3. Select the extracted folder (the one containing `draw_bot.py`).

### 5. Install the Requirements
1. In VS Code, open the top menu and click **Terminal > New Terminal**. A terminal panel will open at the bottom.
2. Type this exact command and hit Enter:
   ```bash
   pip install -r req.txt
   ```
3. Wait for it to download and install libraries like `numpy`, `mediapipe`, and `opencv`.

### 6. Flash the Robot (ESP8266 or ESP32)
*Skip this if your robot is already flashed.*

**If using an ESP8266:**
1. Download Arduino IDE from [arduino.cc/en/software](https://www.arduino.cc/en/software).
2. Open IDE, go to **File > Preferences**, paste this into "Additional Boards Manager URLs": `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
3. Go to **Tools > Board > Boards Manager**, search for `esp8266`, and install it.
4. Plug in your NodeMCU via USB. Go to **Tools > Board** and select `NodeMCU 1.0 (ESP-12E Module)`.
5. Select your COM port, open `draw_bot/draw_bot.ino`, and click **Upload**.

**If using an ESP32:**
1. Open IDE, go to **File > Preferences**, paste this into "Additional Boards Manager URLs": `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Go to **Tools > Board > Boards Manager**, search for `esp32` by Espressif Systems, and install it. **(Important: You must install version 3.0.0 or higher!)**
3. Plug in your ESP32 via USB. Go to **Tools > Board** and select `ESP32 Dev Module`.
4. Select your COM port, open `draw_bot_esp32/draw_bot_esp32.ino`, and click **Upload**.

### 7. Run the Code!
1. Turn on your robot.
2. On your laptop, click the Wi-Fi icon, find the **DrawBot** network, and connect to it (Password: `drawbot123`).
3. Back in VS Code's terminal, type this command and hit Enter:
   ```bash
   python draw_bot.py
   ```
4. A camera window will pop up. Hold up your hand, pinch your thumb and index fingers to draw, and hit **Enter** to make the robot trace it!

---

## Project Files

| File/Folder | Description |
|---|---|
| `draw_bot.py` | **Laptop brain** — camera, hand tracking, path planning, motor math |
| `draw_bot/` | **Robot firmware for ESP8266** — NodeMCU Wi-Fi server + motor control |
| `draw_bot_esp32/` | **Robot firmware for ESP32** — ESP32 Wi-Fi server + motor control |
| `hand_landmarker.task` | Google MediaPipe hand landmark ML model |
| `req.txt` | Python package requirements |
| `tuner.py` | DrawBot Tuner GUI source code |
| `DrawBot Tuner.exe` | Standalone parameter tuning app (no Python needed to run) |
| `code_logic_guide.md` | Deep-dive guide explaining the math and code logic line-by-line |
| `README.md` | This file |

---

## Hardware Requirements

| Component | Details |
|---|---|
| **Microcontroller** | NodeMCU ESP8266 (ESP-12E module) **OR** ESP32 DevKit |
| **Motor Driver** | L298N Dual H-Bridge |
| **Motors** | 2 × DC gear motors (with wheels attached) |
| **Power** | Battery pack for motors (6V–12V), USB for Microcontroller |
| **Laptop** | Any laptop with a webcam and Wi-Fi |
| **Chassis** | Two-wheel differential drive robot chassis |

---

## Wiring Diagrams

### For NodeMCU ESP8266

```
NodeMCU              L298N
───────              ─────
D1  ──────────────►  ENA   (Left motor PWM)
D2  ──────────────►  IN1   (Left motor direction A)
D3  ──────────────►  IN2   (Left motor direction B)

D5  ──────────────►  ENB   (Right motor PWM)
D6  ──────────────►  IN3   (Right motor direction A)
D7  ──────────────►  IN4   (Right motor direction B)

GND ──────────────►  GND   (MUST be common: NodeMCU + L298N + Battery)
```
> **Note:** D3 (GPIO0) is a boot pin on the NodeMCU. If the board ever fails to boot, move the D3 wire to D0 and change the `IN2` define in `draw_bot.ino`.

### For ESP32 (Dev Module)

```
ESP32                 L298N
───────               ─────
GPIO 32 ──────────────► ENA   (Left motor PWM)
GPIO 33 ──────────────► IN1   (Left motor direction A)
GPIO 25 ──────────────► IN2   (Left motor direction B)

GPIO 14 ──────────────► ENB   (Right motor PWM)
GPIO 26 ──────────────► IN3   (Right motor direction A)
GPIO 27 ──────────────► IN4   (Right motor direction B)

GND     ──────────────► GND   (MUST be common: ESP32 + L298N + Battery)
```

---

## Software Requirements

### Python Packages

Install with:
```bash
pip install -r req.txt
```

### ML Model

`hand_landmarker.task` must be in the same folder as `draw_bot.py`.

---

## How to Use

### Drawing Mode

1. Hold your hand in front of the webcam inside the **green box**.
2. **Pinch your thumb and index finger together** = pen down (start drawing).
3. **Separate your fingers** = pen up (stop drawing).
4. Draw your desired path — lines, curves, shapes.
5. Press **Enter** to compile the path.

### Preview Mode

- Green dots show the processed waypoints the robot will follow.
- Green dot with arrow = start position (place your robot here, facing UP).
- Red dot = end position.
- Press **Enter** to upload and start the robot.
- Press **E** to go back and edit.

### Auto Shape Detection

If you draw a shape, the code automatically detects it and replaces your messy drawing with a **mathematically perfect geometric shape**. Look for the red text indicating what it detected:
- Circle
- Triangle
- Rectangle
- Pentagon
- Hexagon

---

## Command Reference

### Normal Use
```bash
python draw_bot.py                          # Open drawing UI
python draw_bot.py --dry-run                # Simulate without talking to the robot
python draw_bot.py --camera 1               # Use a different camera
```

### Calibration Tests (no camera needed)
```bash
python draw_bot.py --calibrate straight     # Drive forward exactly 1 metre
python draw_bot.py --calibrate spin         # Spin exactly 360 degrees on the spot
python draw_bot.py --calibrate square       # Drive a 0.5 m square
python draw_bot.py --calibrate circle       # Drive a perfect circle (r = 0.5 m)
```

### Motor Testing
```bash
python draw_bot.py --ramp                          # Ramp PWM 100→255, find motor dead zone
python draw_bot.py --test-pwm 230 --seconds 3      # Both wheels at PWM 230 for 3 seconds
python draw_bot.py --test-pwm 200 -200             # Left forward, right reverse (spin test)
```

---

## Keyboard Shortcuts

### Drawing Mode (EDIT)
| Key | Action |
|---|---|
| `Enter` | Compile drawing → preview |
| `U` | Undo last stroke |
| `L` | Close the loop (connect last point back to first) |
| `C` | Clear all strokes |
| `Q` | Quit |

### Preview Mode
| Key | Action |
|---|---|
| `Enter` | Upload to robot and GO |
| `E` | Go back to edit mode |
| `C` | Clear and start over |
| `Q` | Quit |

---

## Calibration Guide

Calibrate your robot in this exact order:

### 1. Find PWM_MIN (motor dead zone)
```bash
python draw_bot.py --ramp
```
Watch the terminal and the wheels. Note the PWM value where **both** wheels start turning smoothly. Add 10–15 to that number and set it as `PWM_MIN` in `draw_bot.py` or the Tuner GUI.

### 2. Measure V_MAX (real top speed)
```bash
python draw_bot.py --test-pwm 255 --seconds 3
```
Measure how far the robot travels in 3 seconds (in metres). Divide by 3.
> Example: Travelled 42 cm → V_MAX = 0.42 / 3 = **0.14**
Set `V_CRUISE` to roughly **65–70%** of `V_MAX`.

### 3. Fix straight-line drift (TRIM)
```bash
python draw_bot.py --calibrate straight
```
- Robot drifts **right** → increase `TRIM_LEFT` (e.g., 1.0 → 1.05)
- Robot drifts **left** → increase `TRIM_RIGHT` (e.g., 1.0 → 1.05)

### 4. Calibrate spin angle (SPIN_RATE_DEG_S)
```bash
python draw_bot.py --calibrate spin
```
The robot should spin exactly 360° and stop facing the same way it started.
- **Overshoots** (spins too far) → **increase** `SPIN_RATE_DEG_S`
- **Undershoots** (doesn't spin enough) → **decrease** `SPIN_RATE_DEG_S`

---

## Tuner GUI

Instead of manually editing `draw_bot.py`, use the **DrawBot Tuner**:
- **Double-click `DrawBot Tuner.exe`** — no Python needed.
- **Or run:** `python tuner.py`

### Tuner Parameters
| Slider | What It Does |
|---|---|
| **Trim LEFT** | Compensates if the left motor is stronger/weaker |
| **Trim RIGHT** | Compensates if the right motor is stronger/weaker |
| **Spin Rate (deg/s)** | How fast the robot spins on the spot — calibrates all corner angles |
| **V_MAX (m/s)** | Measured real top speed — must match physical reality |
| **V_CRUISE (m/s)** | Normal driving speed — keep at 65–70% of V_MAX |

---

## Parameter Reference

### Motor Calibration (Critical!)
| Parameter | Default | How to Find |
|---|---|---|
| `WHEEL_BASE_M` | `0.14` | Measure: centre of left wheel to centre of right wheel (metres) |
| `PWM_MIN` | `180` | Run `--ramp`, note where wheels move, add 10 |
| `V_MAX` | `0.20` | Run `--test-pwm 255 --seconds 3`, distance ÷ 3 |
| `V_CRUISE` | `0.15` | Set to approximately 65–70% of V_MAX |
| `SPIN_RATE_DEG_S` | `300.0` | Run `--calibrate spin`, adjust until 360° is exact |
| `TRIM_LEFT` | `1.0` | Fix drift: values above 1 speed the left wheel up |
| `TRIM_RIGHT` | `1.0` | Fix drift: values above 1 speed the right wheel up |

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| "Cannot open the camera" | Wrong camera index | Try `--camera 1` |
| "cannot reach 192.168.4.1" | Not on DrawBot Wi-Fi | Connect laptop to **DrawBot** network (pw: `drawbot123`) |
| Robot doesn't move | PWM too low | Run `--ramp` to find working PWM |
| Robot drifts left or right | Unequal motors | Adjust `TRIM_LEFT` / `TRIM_RIGHT` |
| Corners overshoot or undershoot | Wrong spin rate | Run `--calibrate spin`, adjust `SPIN_RATE_DEG_S` |
| Circle only does 1/4 arc | V_MAX set too high | Measure real V_MAX with `--test-pwm 255 --seconds 3` |
| Curves are jerky | V_CRUISE too close to V_MAX | Set V_CRUISE to 65–70% of V_MAX for headroom |
| ESP32 Compile Error | Old Arduino Core | Update ESP32 board package to >= 3.0.0 |
| "mediapipe crashes on macOS" | mediapipe 1.x bug | Run `pip install mediapipe==0.10.35` |

---

## Wi-Fi & Protocol Quick Reference

| Item | Value |
|---|---|
| SSID | `DrawBot` |
| Password | `drawbot123` |
| IP | `192.168.4.1` |
| Port | `8080` |
| Protocol | ASCII over TCP |

### Commands (Laptop → Robot)
```
CLR                              Clear stored program
S,<left>,<right>,<ms>            Add one motor step
RUN,<count>                      Start playback
STOP                             Emergency stop
```
