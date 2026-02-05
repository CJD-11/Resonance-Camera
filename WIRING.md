# Wiring Reference

Hardware connection guide for the Magnetic Field Camera system.

---

## Components

| Component | Model | I2C Address | Voltage |
|-----------|-------|-------------|---------|
| Single-Board Computer | Raspberry Pi 4 | — | 5V USB-C |
| Camera | Pi Camera Module (CSI) | — | Via CSI bus |
| Magnetometer | BMM150 (Bosch) | 0x13 | 3.3V |

---

## Raspberry Pi 4 GPIO Pinout (Relevant Pins)

```
                    ┌─────────┐
            3.3V  1 │●  ○│ 2   5V
  (SDA1) GPIO2   3 │●  ○│ 4   5V
  (SCL1) GPIO3   5 │●  ○│ 6   GND
                  7 │○  ○│ 8
                    │ .. │
                    │ .. │
                    │ .. │
                    └────┘
```

Only pins 1, 3, 5, and 6 are used for the magnetometer.

---

## BMM150 Magnetometer Wiring

```
    BMM150 Module               Raspberry Pi 4
    ┌────────────┐              ┌──────────────────┐
    │            │              │                  │
    │  VCC ──────┼──────────────┼── Pin 1  (3.3V)  │
    │            │              │                  │
    │  GND ──────┼──────────────┼── Pin 6  (GND)   │
    │            │              │                  │
    │  SDA ──────┼──────────────┼── Pin 3  (GPIO2) │
    │            │              │                  │
    │  SCL ──────┼──────────────┼── Pin 5  (GPIO3) │
    │            │              │                  │
    └────────────┘              └──────────────────┘
```

### Connection Table

| BMM150 Pin | Wire Colour (suggested) | Pi Physical Pin | Pi Function |
|------------|------------------------|-----------------|-------------|
| VCC | Red | Pin 1 | 3.3V power |
| GND | Black | Pin 6 | Ground |
| SDA | Blue | Pin 3 | I2C Data (GPIO2) |
| SCL | Yellow | Pin 5 | I2C Clock (GPIO3) |

### Important Notes

- **Use 3.3V, not 5V.** The BMM150 is a 3.3V device. Connecting to 5V may damage it.
- **No pull-up resistors needed.** The Pi has internal 1.8kΩ pull-ups on SDA and SCL.
- **Keep wires short** (under 30cm) for reliable I2C communication.
- I2C must be enabled via `sudo raspi-config` → Interface Options → I2C → Enable.

---

## Camera Connection

The Pi Camera connects via the CSI (Camera Serial Interface) ribbon cable:

```
    Pi Camera Module
    ┌──────────────┐
    │              │
    │   ┌──────┐   │
    │   │ lens │   │
    │   └──────┘   │
    │              │
    │  ┌────────┐  │
    │  │ribbon  │  │
    └──┤cable   ├──┘
       │        │
       │ (flat  │
       │ ribbon │
       │ cable) │
       │        │
    ┌──┤        ├──────────────────┐
    │  └────────┘    Raspberry Pi  │
    │   CSI connector (between     │
    │   HDMI and audio jack)       │
    └──────────────────────────────┘
```

### Camera Cable Tips

- The cable has a shiny contacts side and a plain blue side
- **Contacts face the HDMI port** on the Pi 4
- Lift the plastic clip on the CSI connector, insert the cable, press the clip back down
- Test with: `libcamera-hello`

---

## Complete System Wiring Diagram

```
                         ┌───────────────────────────────────┐
                         │        RASPBERRY PI 4             │
                         │                                   │
  ┌──────────┐          │  Pin 1 (3.3V) ────── VCC         │
  │ BMM150   │          │  Pin 3 (SDA)  ────── SDA         │──── BMM150
  │ Magneto- │──────────│  Pin 5 (SCL)  ────── SCL         │
  │ meter    │          │  Pin 6 (GND)  ────── GND         │
  └──────────┘          │                                   │
                         │  CSI Port ──── ribbon cable ────  │──── Pi Camera
                         │                                   │
                         │  USB-C ──── power supply ────     │──── 5V 3A
                         │                                   │
                         │  WiFi ──── wlan0 ────             │──── hotspot
                         │              (MagCam network)     │     or home WiFi
                         └───────────────────────────────────┘

  Access from phone/laptop:
    Home WiFi:   http://192.168.1.9:5000
    Hotspot:     http://10.42.0.1:5000
```

---

## Portable Setup

For battery-powered portable use:

```
  ┌─────────────┐     USB-C      ┌──────────┐
  │ USB-C       │────────────────│ Pi 4     │
  │ Battery     │                │          │──── BMM150 (4 wires)
  │ Pack        │                │          │──── Pi Camera (ribbon)
  │ (10000mAh+) │                │          │
  └─────────────┘                └──────────┘
                                      │
                                 WiFi hotspot
                                 (MagCam / magnetic123)
                                      │
                                 ┌──────────┐
                                 │  Phone   │
                                 │ browser  │
                                 │ 10.42.   │
                                 │ 0.1:5000 │
                                 └──────────┘
```

**Battery life estimate:** A 10,000mAh USB-C battery pack powers a Pi 4 + camera + magnetometer for approximately 3–4 hours, depending on processing load and display brightness.

---

## Verification Commands

After wiring, verify everything is connected:

```bash
# Check camera:
libcamera-hello --list-cameras

# Check I2C / magnetometer:
i2cdetect -y 1
# Should show "13" at address 0x13

# Check WiFi interface:
iwconfig wlan0
```

---

*Last Updated: February 2026*
