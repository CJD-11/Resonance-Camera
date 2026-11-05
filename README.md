# Resonance Camera

**Resonance Camera** is an experimental camera that uses environmental measurements to shape photographs. A Raspberry Pi 4 combines a live camera image with visual effects driven by geomagnetic, cosmic-ray, and space-weather data.

The effects are **artistic interpretations of measurements**, not literal pictures of invisible forces or particles.

**Current build:** `resonance_camera_pi4_UI_v8_4_9.py` (`resonance-v8.4.9-standalone-defaults`). This version is designed to run primarily as a self-contained physical camera with an LCD, joystick, and shutter button. A browser interface remains available as an option.

## How it works

```text
Camera image ──────────────────────────┐
                                      ├── Visual processing ── LCD / saved photo
Location → environmental data → cache ┘
```

The camera uses its configured location to select relevant data. Network requests run separately from the image-preview loop, so a slow data service does not need to stall the live image. When a source is unavailable, the program reports that condition rather than treating it as a local measurement.

### Environmental data

| Source | What the camera uses |
| --- | --- |
| **WMM2025** (via `pygeomag`) | A modeled magnetic-field vector for the camera's coordinates. |
| **INTERMAGNET** | Recent magnetic-field readings from a nearby available observatory. **USGS Fredericksburg** is the fallback. |
| **NOAA OVATION** | Aurora forecast values sampled from the grid nearest the camera. |
| **NMDB / Oulu** | Ground-level neutron-monitor measurements used as a regional cosmic-ray proxy. The program tries the available `NEWK` and `OULU` sources and can reuse labeled cached data. |
| **NOAA SWPC / GOES** | Solar-wind speed, interplanetary magnetic-field conditions, Kp, and proton/electron flux. |

Location is taken from explicitly configured coordinates or a previously saved location. If automatic geolocation is enabled, the camera can also estimate its location by IP address. Optional Google Wi-Fi positioning requires an API key and sends nearby access-point information to Google's geolocation service.

**Change from earlier builds:** EPA RadNet and onboard magnetometer/Geiger readings are **not used** in this version.

## Visual modes

Four modes directly use the environmental data:

| Mode | Result |
| --- | --- |
| **Field Plate** | Contours, magnetic-field lines, and measurement annotations. |
| **Field Warp** | Image distortion guided by magnetic-field direction and disturbance. |
| **Cloud Chamber** | Schematic particle tracks whose behavior responds to space-weather and neutron-monitor data. |
| **Resonance Plate** | A more abstract combination of interference, contours, distortions, and particle traces. |

Earlier experimental effects are also available: **Panopticon, Afterimage, Emulsion, Faraday Veil, Erase, Mapped Erase, 4D Hyperspace, Composite, and Raw Camera**. Some of these work best with optional person detection enabled.

The camera image and the effect overlay have **independent black-and-white / color settings**. The default is a black-and-white photograph with a color effect.

## Hardware and controls

The current physical build uses a Raspberry Pi 4, a Pi-compatible camera module, a 320 × 240 SPI TFT display (ILI9340/ILI9341-compatible), an ADS1115-connected analog joystick, a separate shutter button, and an externally switched USB-C power supply. The TFT is **not a touchscreen** in this wiring configuration.

| Control | Action |
| --- | --- |
| Joystick left / right | Change visualization mode. |
| Joystick up / down | Increase / decrease effect intensity. |
| Joystick press | Open settings for image tone and overlay tone. |
| Shutter button | Save a processed high-resolution JPEG. |
| After a photo | Left / right browses saved photos; joystick press returns to live view. |

The LCD shows the live composition and a small control panel. Saved captures default to **1440 × 2560 portrait JPEGs** after rotation. The photo-review screen appears after a shutter capture.

**Default connections (BCM numbering):**

- Shutter: **GPIO27** to GND through a momentary switch.
- Joystick press: **GPIO22**; analog signals use the **ADS1115** on I²C (**GPIO2 SDA / GPIO3 SCL**). The in-script wiring notes connect joystick `VRX → A0` and `VRY → A1`; software reads `A1` as X and `A0` as Y, so check directional behavior with the joystick test.
- TFT: SPI0 **GPIO10 MOSI, GPIO9 MISO, GPIO11 SCK, GPIO8 CE0**, with **GPIO25 DC** and **GPIO24 RESET**. Optional microSD chip select is **GPIO7 CE1**. Check the display board's voltage and backlight requirements before wiring.
- Power: use the **external USB-C switch**. GPIO17 is not used for power control.

See the script's hardware comments for physical-header pin numbers and the full wiring map.

## Running the camera

The program targets Raspberry Pi OS with a supported `Picamera2`/libcamera camera setup and SPI/I²C enabled. Its Python dependencies include **NumPy, OpenCV, Pillow, Flask, Picamera2, RPi.GPIO, `pygeomag`, and the Adafruit Blinka/ADS1115 libraries**. Optional person detection uses **Ultralytics YOLO** and `yolov8n.pt`.

The default LCD backend also requires the separate executable **`resonance_lcd_bridge`**. The script points to `./build_native.sh` for building it; that build file and the bridge are **not part of the attached Python script**. The alternate `adafruit` and `luma` backends require `finearts_lcd_bridge.py`. Install and prepare the appropriate helper before starting the physical display.

Run on the Pi:

```bash
# Override the script's machine-specific default directories if needed.
export CAPTURE_DIR="$HOME/resonance-captures"
export CAMERA_LOCATION_FILE="$HOME/resonance-location.json"
python3 resonance_camera_pi4_UI_v8_4_9.py
```

By default, **`LOCAL_ONLY=1`**: the LCD and physical controls run, but the Flask web server does not start. To turn on the browser interface:

```bash
LOCAL_ONLY=0 python3 resonance_camera_pi4_UI_v8_4_9.py
```

Then open `http://<pi-address>:5000`. Browser preview defaults to **FAST** (the unprocessed camera image); switch to **FX** to see effects in the browser. The physical LCD defaults to showing the effect (`LCD_SOURCE=fx`). Both modes still save processed photographs.

### Useful settings

Set these as environment variables before launching the script:

| Variable | Default | Purpose |
| --- | --- | --- |
| `CAMERA_LAT`, `CAMERA_LON` | Unset | Explicit camera coordinates; recommended for accurate location selection. |
| `GEO_AUTO` | `1` | Enable automatic geolocation fallback. |
| `GEO_WIFI_API_KEY` | Unset | Opt in to Google Wi-Fi geolocation. |
| `LOCAL_ONLY` | `1` | `1`: physical camera only; `0`: enable web interface. |
| `ENABLE_DETECTION` | `0` | Enable optional YOLO person tracking. |
| `LCD_SOURCE` | `fx` | Use `fx` for processed LCD preview or `raw` for a lighter preview. |
| `LCD_BACKEND` | `bb` | Native bridge (`bb`), or the `adafruit` / `luma` helper. |
| `CAPTURE_DIR` | `/home/stoopchild11/captures` | Where captured JPEGs are saved. |
| `CAMERA_LOCATION_FILE` | `/home/stoopchild11/camera_location.json` | Persistent camera coordinates. |
| `PREVIEW_FPS` | `20` | Target camera preview rate (not a guaranteed measured rate). |
| `LCD_FPS` | `8` | Maximum target LCD update rate. |
| `PHOTO_REVIEW_SEC` | `12` | Photo review duration; `0` keeps the review open until dismissed. |

Environmental polling defaults to **20 seconds** for the main update loop, **120 seconds** for OVATION, and **300 seconds** for the neutron-monitor cache. Actual data age depends on each provider.

## Diagnostics

Run a hardware check without starting the regular camera:

```bash
python3 resonance_camera_pi4_UI_v8_4_9.py --lcd-test
python3 resonance_camera_pi4_UI_v8_4_9.py --lcd-benchmark
python3 resonance_camera_pi4_UI_v8_4_9.py --joystick-test
```

With `LOCAL_ONLY=0`, the web interface provides:

- `GET /detection_data` — includes the current environmental snapshot under `environment`, along with preview, tracking, and capture status.
- `GET /environment/location` — reports the current location.
- `POST /environment/location` — saves new coordinates, for example `{"lat": 42.36, "lon": -71.06, "alt_m": 0}`.
- `GET /video_feed` — MJPEG camera preview.

The earlier `/environment_data` endpoint is **not present** in this script. For a local diagnostic export, send **`SIGUSR2`** to the running process; it writes `~/resonance-geo-status.json` by default. **`SIGUSR1`** exports the composed LCD screen as PNG files under `~/resonance-ui-exports/`.

## Limitations

Resonance Camera is an **artistic imaging instrument**, not a calibrated radiation detector, magnetometer, medical device, or safety monitor. WMM values are modeled; observatory and neutron-monitor values may come from distant stations; and GOES describes the wider space environment. No visual mode should be interpreted as showing radiation or magnetic fields literally surrounding the person in the photograph.

Data availability and freshness vary with the network and provider. Person detection, when enabled, is an optional computational effect rather than an environmental measurement.

## Data and attribution

- [World Magnetic Model (NOAA/NCEI)](https://www.ncei.noaa.gov/products/world-magnetic-model)
- [INTERMAGNET](https://intermagnet.org/)
- [USGS Geomagnetism](https://www.usgs.gov/tools/web-service-geomagnetism-data)
- [NOAA Space Weather Prediction Center](https://www.swpc.noaa.gov/)
- [NMDB](https://www.nmdb.eu/) and [University of Oulu cosmic-ray station](https://cosmicrays.oulu.fi/)

Credit NMDB and individual neutron-monitor data providers in accordance with their data-use requirements.
