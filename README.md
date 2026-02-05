# Resonance Camera
-Magnetic Field Camera & Anti-Surveillance System

**An artistic exploration of invisible forces, surveillance, and counter-surveillance using a Raspberry Pi 4, BMM150 magnetometer, and live computer vision.**

![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-c51a4a)
![Python](https://img.shields.io/badge/python-3.9+-blue)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Overview

This project is a portable camera system that visualizes what humans cannot see. It operates in two major modes:

**Magnetic Field Camera** — A live camera feed warped and overlaid by real magnetic field data from a BMM150 3-axis magnetometer. The invisible magnetic field around you becomes visible as pixel distortion, chromatic aberration, curl streamlines, and flux density contours — all rendered in real-time and grounded in actual Maxwell's equations topology.

**Anti-Surveillance Camera** — A YOLO-powered person detection system flipped on its head. Instead of enabling surveillance, it creates artistic visual commentary about observation, identity, and privacy. Six live modes let you watch the camera watch you, dissolve into data, emit privacy shields, or be erased entirely from the scene.

Everything runs on a Raspberry Pi 4 with a Flask web server, accessible from any phone or laptop on the same network. A WiFi hotspot mode makes the system fully portable — no router needed.

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [System Architecture](#system-architecture)
3. [The Scripts](#the-scripts)
4. [Magnetic Field Camera — Modes](#magnetic-field-camera--modes)
5. [Anti-Surveillance Camera — Modes](#anti-surveillance-camera--modes)
6. [Installation](#installation)
7. [WiFi Hotspot (Portable Mode)](#wifi-hotspot-portable-mode)
8. [Usage](#usage)
9. [API Reference](#api-reference)
10. [Troubleshooting](#troubleshooting)
11. [Artist Statement](#artist-statement)
12. [Repository Structure](#repository-structure)
13. [Contact](#contact)

---

## Hardware Requirements

| Component | Model | Quantity | Purpose |
|-----------|-------|----------|---------|
| Single-Board Computer | Raspberry Pi 4 (4GB+) | 1 | Main processing unit |
| Camera | Pi Camera Module / Arducam IMX477 | 1 | Image capture |
| Magnetometer | BMM150 (Bosch) | 1 | 3-axis magnetic field sensing |
| I2C Connection | Jumper wires (SDA, SCL, VCC, GND) | 4 | Sensor communication |
| Power | 5V 3A USB-C supply or battery pack | 1 | Portable power |
| Storage | 16GB+ microSD | 1 | OS and software |

**Optional but recommended:** A portable USB-C battery pack (10,000mAh+) for fully untethered operation.

### BMM150 Wiring

```
BMM150          Raspberry Pi 4
──────          ──────────────
VCC  ──────────  3.3V (Pin 1)
GND  ──────────  GND  (Pin 6)
SDA  ──────────  GPIO2 / SDA1 (Pin 3)
SCL  ──────────  GPIO3 / SCL1 (Pin 5)
```

I2C address: `0x13`

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    MAGNETIC FIELD CAMERA SYSTEM                   │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐    ┌──────────────────────────────────────────┐ │
│  │  BMM150      │    │  PROCESSING PIPELINE                    │ │
│  │  Magnetometer│───▶│                                         │ │
│  │  (I2C 0x13)  │    │  ┌─────────────────┐  ┌─────────────┐  │ │
│  │  20 Hz read  │    │  │ Displacement Map │  │ Overlay     │  │ │
│  └─────────────┘    │  │ Builder (NumPy)  │  │ Renderer    │  │ │
│                      │  │ • Barrel/pinch   │  │ • Curl lines│  │ │
│  ┌─────────────┐    │  │ • Dipole swirl   │  │ • Flux rings│  │ │
│  │  Pi Camera   │───▶│  │ • Curl ripple    │  │ • Vector    │  │ │
│  │  1280×720    │    │  │ • Chromatic sep  │  │   field     │  │ │
│  │  libcamera   │    │  └────────┬────────┘  └──────┬──────┘  │ │
│  └─────────────┘    │           │                   │          │ │
│                      │           ▼                   ▼          │ │
│                      │  ┌────────────────────────────────────┐  │ │
│                      │  │  Frame Compositor + HUD             │  │ │
│                      │  │  → JPEG encode → MJPEG stream      │  │ │
│                      │  └──────────────┬─────────────────────┘  │ │
│                      └─────────────────┼────────────────────────┘ │
│                                        │                          │
│  ┌─────────────────────────────────────▼────────────────────────┐ │
│  │  Flask Web Server (port 5000)                                │ │
│  │  • /video_feed    — MJPEG stream                             │ │
│  │  • /mag_data      — JSON sensor readings + camera metadata   │ │
│  │  • /viz_settings  — Mode & parameter control                 │ │
│  │  • /camera_settings — Exposure, gain, WB, sharpness          │ │
│  │  • Dark-themed responsive UI                                 │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  CLIENT (Phone / Laptop / Tablet)                            │ │
│  │  http://10.42.0.1:5000  (hotspot)                            │ │
│  │  http://192.168.x.x:5000  (home WiFi)                       │ │
│  └──────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

### Anti-Surveillance Pipeline

```
┌──────────────────────────────────────────────────────────────────┐
│                   ANTI-SURVEILLANCE CAMERA                        │
├──────────────────────────────────────────────────────────────────┤
│                                                                    │
│  Camera Frame ──▶ Background Thread ──▶ YOLOv8n ──▶ Detections   │
│       │              (6-7 Hz)            (person    (x1,y1,x2,y2) │
│       │                                  class=0)        │        │
│       │                                                  │        │
│       ▼                                                  ▼        │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                    VISUAL EFFECTS ENGINE                    │  │
│  │                                                             │  │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │  │
│  │  │PANOPTICON│ │  GHOST   │ │DISSOLVE  │ │  SHIELD  │      │  │
│  │  │Scan lines│ │Fade echo │ │Pixelate  │ │Aura rings│      │  │
│  │  │Track IDs │ │Afterimage│ │Data rain │ │Jam rays  │      │  │
│  │  │Biometric │ │NOT FOUND │ │Hex stream│ │Static    │      │  │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘      │  │
│  │                                                             │  │
│  │  ┌──────────┐ ┌──────────┐                                 │  │
│  │  │  ERASED  │ │ALL LAYERS│                                 │  │
│  │  │Inpaint BG│ │Combined  │                                 │  │
│  │  │Green rect│ │          │                                 │  │
│  │  └──────────┘ └──────────┘                                 │  │
│  └─────────────────────────────────────────────────────────────┘  │
│       │                                                           │
│       ▼                                                           │
│  Flask MJPEG Stream ──▶ Phone / Laptop Browser                    │
└──────────────────────────────────────────────────────────────────┘
```

---

## The Scripts

| Script | Description | Dependencies |
|--------|-------------|-------------|
| `magnometer_viz_v2.py` | Magnetic field camera — standard distortion + field overlay | Flask, Picamera2, PIL, NumPy, smbus2 |
| `magnometer_viz_v3.py` | Magnetic field camera — **dramatic** physics visualization (curl, chromatic aberration, dipole vortex) | Same as v2 |
| `antisurveillance_camera.py` | Anti-surveillance art — 6 live visual modes using YOLO person detection | Flask, Picamera2, PIL, NumPy, OpenCV, ultralytics |
| `setup_hotspot.sh` | One-time WiFi hotspot setup for portable use | NetworkManager (nmcli) |
| `wifi_switch.sh` | Quick toggle between hotspot mode and home WiFi | NetworkManager (nmcli) |

---

## Magnetic Field Camera — Modes

### V2 (Standard)

| Mode | Description |
|------|-------------|
| **Original** | Clean camera feed with HUD showing live magnetometer readings |
| **Distorted** | Barrel/pincushion + swirl + directional shift driven by BMM150 X/Y/Z |
| **Field Overlay** | Distortion + 32 curved radial field lines + 20×20 directional tick grid |

### V3 (Dramatic / Physics-Based)

| Mode | Description |
|------|-------------|
| **Original** | Clean feed with enhanced HUD (∇×B and ∇·B readouts) |
| **Distorted** | 4th-order barrel distortion + dipole counter-rotating vortex + curl ripple waves + chromatic aberration (RGB channel separation) + animated phase drift + edge vignette |
| **Field Overlay** | Distortion + 48 curl streamlines (∇×B) + deformed equipotential rings + 28×28 dipole vector field with proper arrowheads + divergence glow at center |

### Camera Controls

Both versions include full manual camera control:

- **Image:** Brightness, Contrast, Saturation, Sharpness
- **Exposure:** Auto/Manual toggle, Exposure Time (0.1ms–200ms), Analogue Gain (1×–16×), EV Compensation (±8 stops)
- **White Balance:** Auto, Tungsten, Fluorescent, Indoor, Daylight, Cloudy
- **Live Sensor Readout:** Shows actual exposure time and gain from hardware metadata

> **Technical note:** All manual exposure controls (AeEnable, ExposureTime, AnalogueGain) are sent as a single bundled `set_controls()` call — this is required by libcamera to prevent the auto-exposure algorithm from overriding individual settings.

---

## Anti-Surveillance Camera — Modes

| Mode | Visual Effect | Conceptual Statement |
|------|---------------|---------------------|
| **Panopticon** | Surveillance grid, sweeping scan line, tracking boxes with rotating subject IDs, biometric scan lines, red crosshairs, REC timestamp | *The camera watches you.* |
| **Ghost** | Detected people fade to translucent echoes; past positions leave green-tinted afterimages that decay over time; "NOT FOUND" flickers | *The surveilled dissolve. You cannot capture what is already gone.* |
| **Dissolution** | Progressive pixelation (fine→coarse top to bottom), static noise injection, green data-rain streams, hex dump streaming alongside the body | *Identity breaks apart into data.* |
| **Field Shield** | Pulsing deformed privacy-aura rings (cyan/green/blue hue shift), interference rays, central shield glyph, screen-wide static noise | *The body fights back against the camera.* |
| **Erased** | OpenCV TELEA inpainting fills the background where the person stood; green outlined rectangle with corner accents marks the absence | *The person is removed. Only the trace remains.* |
| **All Layers** | Every effect combined at reduced intensity | *Everything at once.* |

---

## Installation

### 1. System Dependencies

```bash
sudo apt update
sudo apt install -y python3-pip python3-picamera2 python3-pil python3-numpy python3-opencv
```

### 2. Python Packages

```bash
pip3 install flask smbus2 --break-system-packages

# For anti-surveillance camera only:
pip3 install ultralytics --break-system-packages
```

### 3. Enable I2C (for magnetometer)

```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
sudo reboot
```

Verify sensor:

```bash
i2cdetect -y 1
# Should show device at address 0x13
```

### 4. Transfer Scripts to Pi

From your computer (Git Bash / Terminal):

```bash
scp magnometer_viz_v2.py stoopchild11@192.168.1.9:~/
scp magnometer_viz_v3.py stoopchild11@192.168.1.9:~/
scp antisurveillance_camera.py stoopchild11@192.168.1.9:~/
scp setup_hotspot.sh stoopchild11@192.168.1.9:~/
scp wifi_switch.sh stoopchild11@192.168.1.9:~/
```

### 5. Run

```bash
# Magnetic field camera (standard):
python3 ~/magnometer_viz_v2.py

# Magnetic field camera (dramatic physics):
python3 ~/magnometer_viz_v3.py

# Anti-surveillance camera:
python3 ~/antisurveillance_camera.py
```

Access from any browser on the same network: `http://192.168.1.9:5000`

---

## WiFi Hotspot (Portable Mode)

Makes the Pi broadcast its own WiFi network so you can use it anywhere without a router.

### One-Time Setup

```bash
sudo bash ~/setup_hotspot.sh
```

### Connecting

1. Open WiFi settings on your phone
2. Connect to: **MagCam**
3. Password: **magnetic123**
4. Browse to: **http://10.42.0.1:5000**

### Switching Between Modes

```bash
sudo bash ~/wifi_switch.sh hotspot   # Broadcast WiFi
sudo bash ~/wifi_switch.sh wifi      # Reconnect to home network
sudo bash ~/wifi_switch.sh status    # Show active connections
```

> **Note:** The Pi cannot be on home WiFi and hotspot simultaneously. SSH from your computer won't work in hotspot mode — use your phone or set up a systemd auto-start service.

### Auto-Start Service (Recommended for Portable Use)

```bash
sudo nano /etc/systemd/system/magcam.service
```

```ini
[Unit]
Description=Magnetic Field Camera
After=network.target

[Service]
User=stoopchild11
WorkingDirectory=/home/stoopchild11
ExecStart=/usr/bin/python3 /home/stoopchild11/magnometer_viz_v3.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable magcam.service
sudo systemctl start magcam.service
```

---

## API Reference

All scripts share a similar Flask API structure:

### Magnetic Field Camera Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serves the web interface |
| `/video_feed` | GET | MJPEG stream of processed frames |
| `/mag_data` | GET | JSON: `{x, y, z, magnitude, cam_exposure, cam_gain}` |
| `/viz_settings` | POST | Set `mode`, `distortion_strength`, `opacity` |
| `/camera_settings` | POST | Set any camera control |
| `/camera_info` | GET | Debug: full camera metadata dump |
| `/capture` | GET | Save current frame as JPEG |

### Anti-Surveillance Camera Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serves the web interface |
| `/video_feed` | GET | MJPEG stream of processed frames |
| `/detection_data` | GET | JSON: `{count, total_seen, people}` |
| `/viz_settings` | POST | Set `mode`, `intensity`, `ghost_persistence`, `scan_speed` |
| `/capture` | GET | Save current processed frame as JPEG |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `No cameras available` | Check CSI ribbon cable. Run `libcamera-hello` to test. |
| BMM150 not at 0x13 | `i2cdetect -y 1` — check SDA→GPIO2, SCL→GPIO3, VCC→3.3V |
| Port 5000 in use | `pkill -f magnometer` or `pkill -f antisurveillance` then restart |
| Exposure sliders no effect | Turn OFF Auto Exposure. Check Sensor Readout cards for confirmation. |
| YOLO model won't download | Manual: `wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt` |
| Hotspot not appearing | Verify: `nmcli connection show`. Re-run `sudo bash ~/setup_hotspot.sh`. |
| SSH dies after hotspot | Expected — Pi left your WiFi. Connect phone to MagCam network instead. |
| Low frame rate | Reduce resolution in script config, lower distortion strength, or use `imgsz=320` for YOLO. |
| Pasting into nano kills terminal | Don't paste large files — use SCP instead. |

---



## Repository Structure

```
Magnetic-Field-Camera/
├── README.md                          # This file
├── magnometer_viz_v2.py               # Magnetic field camera (standard)
├── magnometer_viz_v3.py               # Magnetic field camera (dramatic physics)
├── antisurveillance_camera.py         # Anti-surveillance art installation
├── setup_hotspot.sh                   # WiFi hotspot one-time setup
├── wifi_switch.sh                     # Hotspot ↔ WiFi toggle
├── docs
│   ├── SETUP_GUIDE.md                 # Detailed installation walkthrough
│   ├── ARCHITECTURE.md                # Technical deep-dive
│   └── WIRING.md                      # Hardware connection reference
└── reference/
    └── MagneticFieldCamera_Reference.pdf  # Complete offline reference
```

---

## Performance

| Configuration | FPS | Notes |
|---------------|-----|-------|
| Mag Camera V2 @ 1280×720 | 8–12 | Distortion + overlay |
| Mag Camera V3 @ 1280×720 | 5–8 | Heavier math (curl, chromatic) |
| Anti-Surveillance @ 640×480 | 4–6 | YOLO detection in background thread |
| Anti-Surveillance Erased mode | 2–4 | Inpainting is CPU-intensive |

---

## Contact

**Corey Dziadzio**
- 📧 Email: coreydziadzio@coreydziadzio.com
- 🌐 Website: (https://www.coreydziadzio.com/)
- 🔗 GitHub: [@CJD-11](https://github.com/CJD-11)

---

*Last Updated: February 2026*
