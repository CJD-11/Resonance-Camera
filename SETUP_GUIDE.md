# Setup Guide

Complete step-by-step instructions for building and deploying the Magnetic Field Camera and Anti-Surveillance Camera system on a Raspberry Pi 4.

---

## Prerequisites

**You need:**
- Raspberry Pi 4 (4GB or 8GB recommended) with Raspberry Pi OS installed
- Pi Camera Module (any compatible model) connected via CSI ribbon cable
- BMM150 magnetometer module (for magnetic field scripts)
- A computer on the same network for file transfer
- SSH enabled on the Pi

**Your Pi's IP address** (used throughout this guide): `192.168.1.9`
*(Find yours with `hostname -I` on the Pi)*

---

## Step 1: System Packages

SSH into your Pi and run:

```bash
sudo apt update
sudo apt install -y python3-pip python3-picamera2 python3-pil python3-numpy python3-opencv
```

---

## Step 2: Python Libraries

```bash
pip3 install flask smbus2 --break-system-packages
```

For the Anti-Surveillance Camera (YOLO person detection):

```bash
pip3 install ultralytics --break-system-packages
```

> **Note:** The first time YOLO runs, it will download the model file (~6MB). This requires internet access on the Pi.

---

## Step 3: Enable I2C (Magnetometer Only)

```bash
sudo raspi-config
```

Navigate to: **Interface Options → I2C → Enable**

Reboot:

```bash
sudo reboot
```

After reboot, verify the BMM150 is detected:

```bash
i2cdetect -y 1
```

You should see `13` in the grid (address 0x13).

---

## Step 4: Wire the BMM150

| BMM150 Pin | Pi Pin | Pi GPIO |
|------------|--------|---------|
| VCC | Pin 1 | 3.3V |
| GND | Pin 6 | Ground |
| SDA | Pin 3 | GPIO2 (SDA1) |
| SCL | Pin 5 | GPIO3 (SCL1) |

That's it — four wires. No resistors, no level shifting needed (BMM150 is 3.3V native).

---

## Step 5: Transfer Scripts to the Pi

From your computer's terminal (**type** these commands, don't paste if using Git Bash):

```bash
scp magnometer_viz_v2.py stoopchild11@192.168.1.9:~/magnometer_viz_v2.py
scp magnometer_viz_v3.py stoopchild11@192.168.1.9:~/magnometer_viz_v3.py
scp antisurveillance_camera.py stoopchild11@192.168.1.9:~/antisurveillance_camera.py
scp setup_hotspot.sh stoopchild11@192.168.1.9:~/setup_hotspot.sh
scp wifi_switch.sh stoopchild11@192.168.1.9:~/wifi_switch.sh
```

> **Why type instead of paste?** Git Bash on Windows uses "bracketed paste mode" which can prepend garbage characters (`^[[200~`) to pasted commands.

---

## Step 6: Run a Script

SSH into the Pi:

```bash
ssh stoopchild11@192.168.1.9
```

Kill any previously running camera script:

```bash
pkill -f magnometer
pkill -f antisurveillance
```

Start the one you want:

```bash
# Magnetic field camera (standard):
python3 ~/magnometer_viz_v2.py

# Magnetic field camera (dramatic physics):
python3 ~/magnometer_viz_v3.py

# Anti-surveillance camera:
python3 ~/antisurveillance_camera.py
```

Then open a browser on any device on the same network and go to:

```
http://192.168.1.9:5000
```

---

## Step 7: WiFi Hotspot (Optional — For Portable Use)

This makes the Pi broadcast its own WiFi network so you can use it anywhere.

### First Time Setup

While still SSH'd in:

```bash
sudo bash ~/setup_hotspot.sh
```

**Your SSH session will disconnect** — that's expected. The Pi is now a hotspot.

### Connect Your Phone

1. Open WiFi settings
2. Connect to: **MagCam**
3. Password: **magnetic123**
4. Open browser: **http://10.42.0.1:5000**

### Switching Back to Home WiFi

Connect a keyboard and monitor to the Pi (or use Raspberry Pi Connect), then:

```bash
sudo bash ~/wifi_switch.sh wifi
```

---

## Step 8: Auto-Start on Boot (Recommended)

So you don't need SSH to start the camera:

```bash
sudo nano /etc/systemd/system/magcam.service
```

Paste this:

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

> Change the `ExecStart` path to whichever script you want to auto-run.

Enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable magcam.service
sudo systemctl start magcam.service
```

Check it's running:

```bash
sudo systemctl status magcam.service
```

View logs:

```bash
journalctl -u magcam.service -f
```

---

## Step 9: Create Captures Directory

```bash
mkdir -p ~/captures
```

All scripts save captured photos here when you press the Capture button in the web UI.

---

## Updating a Script

If you edit a script on your computer and need to re-deploy:

```bash
# From your computer:
scp ~/Downloads/magnometer_viz_v3.py stoopchild11@192.168.1.9:~/magnometer_viz_v3.py

# On the Pi (if running manually):
pkill -f magnometer
python3 ~/magnometer_viz_v3.py

# On the Pi (if using systemd):
sudo systemctl restart magcam.service
```

---

## Increasing Swap Space (If Running Out of Memory)

YOLO can be memory-hungry on a 4GB Pi:

```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
# Change CONF_SWAPSIZE=100 to CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

---

*Last Updated: February 2026*
