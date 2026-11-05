import io
import sys
import subprocess
import os
import math
import time
import threading
import signal
import queue
import random
import re
import json
import statistics
import collections as _collections
import urllib.parse
import urllib.request
import colorsys
import cv2
import numpy as np
from datetime import datetime, timezone, timedelta
from pathlib import Path
from flask import Flask, Response, render_template_string, jsonify, request
from picamera2 import Picamera2
from PIL import Image, ImageDraw, ImageFont, ImageEnhance, ImageOps, ImageFilter

SCRIPT_VERSION = "resonance-v8.4.9-standalone-defaults"

# Default stand-alone Pi 4 profile: optimized LCD FX, no browser/detection,
# auto geographic location and cosmic particle refresh. All can still be
# overridden by environment variables from a shell or systemd service.


# ─── GPIO Setup ───────────────────────────────────────────────────────────────
# Pi power is controlled by the user's external USB-C power switch.
# GPIO17 / physical pin 11 is intentionally unused in this build.
try:
    from RPi import GPIO

    BUTTON_PIN = int(os.environ.get("SHUTTER_PIN", "27"))          # GPIO27 / physical pin 13
    JOYSTICK_SW_PIN = int(os.environ.get("JOYSTICK_SW_PIN", "22")) # GPIO22 / physical pin 15

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(JOYSTICK_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    HAS_GPIO = True
    print(
        f"✓ GPIO ready — shutter GPIO{BUTTON_PIN}, joystick switch GPIO{JOYSTICK_SW_PIN}; "
        "GPIO17 unused (Pi power is switched at USB-C)"
    )
except Exception as e:
    print(f"GPIO not available: {e}")
    HAS_GPIO = False

# ─── Local 10-pin TFT + ADS1115 Joystick Hardware ────────────────────────────
# LCD connector exposed by this build:
#   GND, VIN, DC, RST, SDCS, CS, MOSI, MISO, SCK, BL
#
# IMPORTANT:
# Those 10 pins expose SPI display/microSD/backlight only. There is no touch
# controller connection in this pin list, so touchscreen input is intentionally
# disabled in this script. The local UI is controlled by the joystick and the
# dedicated shutter; the phone/web UI still works normally.
#
# 10-pin LCD wiring:
#   GND   -> Pi GND / physical pin 6
#   VIN   -> Pi 3.3V / physical pin 1 (Product 1480 accepts 3-5V; this build uses 3.3V)
#   DC    -> GPIO25 / physical pin 22
#   RST   -> GPIO24 / physical pin 18
#   SDCS  -> GPIO7 / CE1 / physical pin 26 (optional; microSD only)
#   CS    -> GPIO8 / CE0 / physical pin 24
#   MOSI  -> GPIO10 / physical pin 19
#   MISO  -> GPIO9 / physical pin 21 (optional for display-only; used by microSD/readback)
#   SCK   -> GPIO11 / physical pin 23
#   BL    -> leave disconnected initially unless the exact LCD board requires
#            a logic/power connection for backlight control.
#
# ADS1115 + HiLetgo 5-wire joystick:
#   ADS1115 VDD -> Pi 3.3V / physical pin 17
#   ADS1115 GND -> Pi GND / physical pin 20
#   ADS1115 SDA -> GPIO2 / physical pin 3
#   ADS1115 SCL -> GPIO3 / physical pin 5
#   Joystick VCC -> ADS1115 VDD / 3.3V
#   Joystick GND -> ADS1115 GND
#   Joystick VRX -> ADS1115 A0
#   Joystick VRY -> ADS1115 A1
#   Joystick SW  -> GPIO22 / physical pin 15
#
# Dedicated shutter:
#   GPIO27 / physical pin 13 -> momentary switch -> GND / physical pin 14
#
# Pi itself remains powered only through the switched USB-C connection.

ENABLE_LOCAL_LCD = os.environ.get("ENABLE_LOCAL_LCD", "1") == "1"
ENABLE_JOYSTICK = os.environ.get("ENABLE_JOYSTICK", "1") == "1"
# LCD_BACKEND chooses the implementation, not the physical controller type.
# Product #1480 is ILI9340 / ILI9341-compatible, not ST7789.
LCD_DRIVER = os.environ.get("LCD_DRIVER", "ILI9340").upper()  # legacy metadata
LCD_BACKEND = os.environ.get("LCD_BACKEND", "bb").strip().lower()
if LCD_BACKEND not in ("bb", "adafruit", "luma"):
    raise ValueError("LCD_BACKEND must be bb, adafruit or luma")
LCD_ROTATION = int(os.environ.get("LCD_ROTATION", "90")) % 360
LCD_SOFTWARE_FLIP_180 = os.environ.get("LCD_SOFTWARE_FLIP_180", "1") == "1"
LCD_FPS = float(np.clip(float(os.environ.get("LCD_FPS", "8")), 1.0, 8.0))
LCD_BAUDRATE = int(os.environ.get("LCD_BAUDRATE", "4000000"))
LCD_DC_PIN = int(os.environ.get("LCD_DC_PIN", "25"))
LCD_RESET_PIN = int(os.environ.get("LCD_RESET_PIN", "24"))
ADS1115_ADDR = int(os.environ.get("ADS1115_ADDR", "0x48"), 0)
JOYSTICK_INVERT_X = os.environ.get("JOYSTICK_INVERT_X", "0") == "1"
JOYSTICK_INVERT_Y = os.environ.get("JOYSTICK_INVERT_Y", "0") == "1"
JOYSTICK_LOW_V = float(os.environ.get("JOYSTICK_LOW_V", "0.85"))
JOYSTICK_HIGH_V = float(os.environ.get("JOYSTICK_HIGH_V", "2.45"))
JOYSTICK_DEBOUNCE = float(os.environ.get("JOYSTICK_DEBOUNCE", "0.08"))
JOYSTICK_DEADZONE_V = float(os.environ.get("JOYSTICK_DEADZONE_V", "0.38"))
LCD_SELF_TEST = os.environ.get("LCD_SELF_TEST", "0") == "1"
LCD_BENCHMARK = "--lcd-benchmark" in sys.argv
LCD_ONLY_TEST = LCD_BENCHMARK or ("--lcd-test" in sys.argv) or (os.environ.get("LCD_ONLY_TEST", "0") == "1")
JOYSTICK_ONLY_TEST = ("--joystick-test" in sys.argv) or (os.environ.get("JOYSTICK_ONLY_TEST", "0") == "1")
LCD_MENU_TIMEOUT_SEC = float(os.environ.get("LCD_MENU_TIMEOUT_SEC", "8"))
LCD_TEST_HOLD_SEC = float(os.environ.get("LCD_TEST_HOLD_SEC", "1.0"))
LCD_BRIDGE_BIN = os.environ.get(
    "LCD_BRIDGE_BIN",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "resonance_lcd_bridge"),
)
# Native SPI rate verified previously on this camera at 16 MHz.
LCD_NATIVE_SPI_HZ = int(os.environ.get("LCD_NATIVE_SPI_HZ", "16000000"))
LCD_NATIVE_CS = int(os.environ.get("LCD_NATIVE_CS", "8"))
LCD_NATIVE_DC = int(os.environ.get("LCD_NATIVE_DC", "25"))
LCD_NATIVE_RST = int(os.environ.get("LCD_NATIVE_RST", "24"))
LCD_SWAP565 = int(os.environ.get("LCD_SWAP565", "1"))

HAS_LCD = False
HAS_JOYSTICK = False
_lcd = None  # compatibility marker; native bridge is authoritative
_ads = None
_joy_x = None
_joy_y = None
_lcd_width = 320
_lcd_height = 240
_lcd_native_width = 240
_lcd_native_height = 320
_local_hw_i2c = None

# Native bb_spi_lcd bridge.  Python renders 320x240 RGB frames; the helper
# converts them to RGB565 and writes them through /dev/spidev0.0.
_lcd_bridge = None
_lcd_write_lock = threading.Lock()
_lcd_last_error = None
_lcd_write_count = 0

_shutdown_event = threading.Event()


def _board_pin(board_mod, bcm_pin):
    """Return a Blinka board.Dxx pin from a BCM number (joystick/I2C only)."""
    return getattr(board_mod, f"D{int(bcm_pin)}")


def _stop_native_lcd_bridge():
    global _lcd_bridge
    proc = _lcd_bridge
    _lcd_bridge = None
    if proc is None:
        return
    try:
        if proc.stdin:
            proc.stdin.close()
    except Exception:
        pass
    try:
        proc.wait(timeout=1.5)
    except Exception:
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def _start_native_lcd_bridge():
    """Start the proven bb_spi_lcd Linux backend and wait for its READY line."""
    global _lcd_bridge, HAS_LCD, _lcd_last_error
    _stop_native_lcd_bridge()

    # Python alternatives implement the same acknowledged 320x240 RGB888 pipe
    # protocol, so NONE of the fine-arts display/menu logic needs replacing.
    if LCD_BACKEND == "bb":
        bridge = str(LCD_BRIDGE_BIN)
        if not os.path.isfile(bridge):
            raise FileNotFoundError(
                f"Native LCD bridge not found: {bridge}. "
                "Run ./build_native.sh or set LCD_BRIDGE_BIN."
            )
        if not os.access(bridge, os.X_OK):
            raise PermissionError(f"Native LCD bridge is not executable: {bridge}")
        cmd = [bridge]
    else:
        helper = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "finearts_lcd_bridge.py")
        if not os.path.isfile(helper):
            raise FileNotFoundError(f"LCD Python driver helper missing: {helper}")
        cmd = [sys.executable, "-u", helper, "--backend", LCD_BACKEND]

    cmd += [
        "--speed", str(int(LCD_NATIVE_SPI_HZ)),
        "--rotation", str(int(LCD_ROTATION)),
        "--cs", str(int(LCD_NATIVE_CS)),
        "--dc", str(int(LCD_NATIVE_DC)),
        "--rst", str(int(LCD_NATIVE_RST)),
        "--swap565", str(int(LCD_SWAP565)),
    ]

    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=None,     # bridge diagnostics go directly to journal/terminal
        bufsize=0,
    )

    ready = proc.stdout.readline().decode("utf-8", "replace").strip() if proc.stdout else ""
    if proc.poll() is not None:
        raise RuntimeError(f"Native LCD bridge exited during startup (status {proc.returncode}): {ready}")
    if not ready.startswith("READY V8_4_2 "):
        try:
            proc.terminate()
        except Exception:
            pass
        raise RuntimeError(
            f"Wrong/outdated LCD bridge. Expected v8.4.2 compatible pipe protocol, got: {ready!r}"
        )

    parts = ready.split()
    # READY V8_4_2 <logical_w> <logical_h> <native_w> <native_h>
    if len(parts) >= 6:
        lw, lh = int(parts[2]), int(parts[3])
        if (lw, lh) != (_lcd_width, _lcd_height):
            raise RuntimeError(
                f"LCD bridge geometry {lw}x{lh} does not match UI "
                f"{_lcd_width}x{_lcd_height}"
            )

    _lcd_bridge = proc
    HAS_LCD = True
    _lcd_last_error = None
    print(
        f"✓ TFT bridge ready: {LCD_BACKEND} → /dev/spidev0.0, "
        f"logical={_lcd_width}x{_lcd_height}, native={_lcd_native_width}x{_lcd_native_height}, "
        f"rotation={LCD_ROTATION}, SPI={LCD_NATIVE_SPI_HZ/1_000_000:.1f}MHz"
    )


def _lcd_pipe_write_all(payload):
    """Write one complete RGB888 frame to the native bridge.

    subprocess stdin is a pipe.  Large writes must not be assumed to complete
    in a single call, especially during continuous live preview.
    """
    if _lcd_bridge is None or _lcd_bridge.stdin is None:
        raise BrokenPipeError("native LCD bridge stdin is unavailable")

    view = memoryview(payload)
    total = 0
    while total < len(view):
        n = _lcd_bridge.stdin.write(view[total:])
        if n is None:
            # Buffered streams may return None; flush and retry.
            _lcd_bridge.stdin.flush()
            continue
        if n <= 0:
            raise BrokenPipeError(
                f"native LCD bridge accepted {total}/{len(view)} bytes"
            )
        total += int(n)

    try:
        _lcd_bridge.stdin.flush()
    except Exception:
        pass
    return total


def _lcd_write_image(image, _retry=True):
    """Send one complete logical RGB888 frame and wait for bridge ACK.

    The C++ bridge acknowledges only after it has converted the frame to
    display-order RGB565 and called bb_spi_lcd::pushImage().
    """
    global _lcd_write_count, _lcd_last_error
    if not HAS_LCD or _lcd_bridge is None:
        return False

    img = image.convert("RGB")
    expected = (_lcd_width, _lcd_height)
    if img.size != expected:
        img = ImageOps.fit(
            img, expected, method=Image.Resampling.BILINEAR,
            centering=(0.5, 0.5)
        )

    # Keep the ILI9340/ILI9341 controller in its proven-good 90° mode.
    # If the mounted panel appears upside down, rotate the finished logical
    # 320x240 frame in software instead of using controller rotation 270°.
    if LCD_SOFTWARE_FLIP_180:
        img = img.transpose(Image.Transpose.ROTATE_180)

    payload = img.tobytes()
    expected_bytes = _lcd_width * _lcd_height * 3
    if len(payload) != expected_bytes:
        raise RuntimeError(
            f"LCD frame byte count {len(payload)} != expected {expected_bytes}"
        )

    try:
        with _lcd_write_lock:
            if _lcd_bridge is None or _lcd_bridge.poll() is not None:
                raise BrokenPipeError("native LCD bridge is not running")

            written = _lcd_pipe_write_all(payload)
            if written != expected_bytes:
                raise IOError(
                    f"short LCD frame write: {written}/{expected_bytes} bytes"
                )

            ack_raw = _lcd_bridge.stdout.readline() if _lcd_bridge.stdout else b""
            if not ack_raw:
                raise BrokenPipeError("LCD bridge closed before frame ACK")
            ack = ack_raw.decode("utf-8", "replace").strip()
            if not ack.startswith("FRAME_OK "):
                raise RuntimeError(f"unexpected LCD bridge response: {ack!r}")

            _lcd_write_count += 1
            _lcd_last_error = None
        return True

    except (BrokenPipeError, OSError, IOError, RuntimeError) as exc:
        _lcd_last_error = str(exc)
        print(f"Native TFT bridge write failed: {exc}")
        if _retry and not _shutdown_event.is_set():
            try:
                print("Restarting native TFT bridge...")
                _start_native_lcd_bridge()
                return _lcd_write_image(img, _retry=False)
            except Exception as restart_exc:
                _lcd_last_error = str(restart_exc)
                print(f"Native TFT bridge restart failed: {restart_exc}")
        return False


def _lcd_write_solid(rgb, hold=0.0):
    """Diagnostic solid fill sent through the same native bridge as live frames."""
    img = Image.new("RGB", (_lcd_width, _lcd_height), tuple(int(v) for v in rgb))
    ok = _lcd_write_image(img)
    if ok and hold > 0:
        time.sleep(hold)
    return ok


def _lcd_checkerboard(hold=0.0):
    if not HAS_LCD:
        return False
    img = Image.new("RGB", (_lcd_width, _lcd_height), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    cell = max(16, min(_lcd_width, _lcd_height) // 6)
    for y in range(0, _lcd_height, cell):
        for x in range(0, _lcd_width, cell):
            if ((x // cell) + (y // cell)) % 2 == 0:
                draw.rectangle(
                    (x, y, min(_lcd_width - 1, x + cell - 1),
                     min(_lcd_height - 1, y + cell - 1)),
                    fill=(255, 255, 255)
                )
    ok = _lcd_write_image(img)
    if ok and hold > 0:
        time.sleep(hold)
    return ok


def _run_lcd_diagnostic(long_test=False):
    if not HAS_LCD:
        print("LCD diagnostic skipped: native bridge did not initialize.")
        return False
    hold = LCD_TEST_HOLD_SEC if long_test else 0.20
    print(f"TFT diagnostic through {LCD_BACKEND}: RED → GREEN → BLUE → WHITE → BLACK")
    for name, rgb in (
        ("RED", (255, 0, 0)),
        ("GREEN", (0, 255, 0)),
        ("BLUE", (0, 0, 255)),
        ("WHITE", (255, 255, 255)),
        ("BLACK", (0, 0, 0)),
    ):
        print(f"  TFT frame: {name}")
        if not _lcd_write_solid(rgb, hold=hold):
            return False

    print("TFT diagnostic: checkerboard")
    if not _lcd_checkerboard(hold=hold):
        return False

    print("TFT diagnostic: landscape orientation quadrants")
    orient = Image.new("RGB", (_lcd_width, _lcd_height), (0, 0, 0))
    od = ImageDraw.Draw(orient)
    mx, my = _lcd_width // 2, _lcd_height // 2
    od.rectangle((0, 0, mx - 1, my - 1), fill=(255, 0, 0))
    od.rectangle((mx, 0, _lcd_width - 1, my - 1), fill=(0, 255, 0))
    od.rectangle((0, my, mx - 1, _lcd_height - 1), fill=(0, 0, 255))
    od.rectangle((mx, my, _lcd_width - 1, _lcd_height - 1), fill=(255, 255, 255))
    od.text((8, 8), "TL", fill=(255, 255, 255))
    od.text((mx + 8, 8), "TR", fill=(0, 0, 0))
    od.text((8, my + 8), "BL", fill=(255, 255, 255))
    od.text((mx + 8, my + 8), "BR", fill=(0, 0, 0))
    if not _lcd_write_image(orient):
        return False
    if hold > 0:
        time.sleep(hold)

    print("✓ Native TFT diagnostic completed.")
    return True


def _init_local_hardware():
    """Initialize native LCD first; use Blinka only for ADS1115 joystick I2C."""
    global HAS_LCD, HAS_JOYSTICK
    global _ads, _joy_x, _joy_y, _local_hw_i2c

    if ENABLE_LOCAL_LCD and not JOYSTICK_ONLY_TEST:
        try:
            _start_native_lcd_bridge()
            if LCD_SELF_TEST and not LCD_ONLY_TEST:
                _run_lcd_diagnostic(long_test=False)
        except Exception as exc:
            HAS_LCD = False
            print(f"Native TFT unavailable: {type(exc).__name__}: {exc}")

    # --lcd-test is intentionally LCD-only; don't initialize Blinka/I2C.
    if LCD_ONLY_TEST or not ENABLE_JOYSTICK:
        return

    try:
        import board as board_mod
        _local_hw_i2c = board_mod.I2C()
    except Exception as exc:
        print(f"Joystick I2C unavailable: {exc}")
        _local_hw_i2c = None
        return

    try:
        import adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15 import ads1x15
        from adafruit_ads1x15.analog_in import AnalogIn

        _ads = ADS.ADS1115(_local_hw_i2c, address=ADS1115_ADDR)
        _ads.gain = 1
        _joy_x = AnalogIn(_ads, ads1x15.Pin.A1)
        _joy_y = AnalogIn(_ads, ads1x15.Pin.A0)
        HAS_JOYSTICK = True
        print(
            f"✓ Joystick ADC ready: ADS1115 0x{ADS1115_ADDR:02X}, "
            f"X=A1 Y=A0, push=GPIO{JOYSTICK_SW_PIN}"
        )
    except Exception as exc:
        HAS_JOYSTICK = False
        print(f"Joystick ADC unavailable: {type(exc).__name__}: {exc}")


_init_local_hardware()

# ─── Official / Research Environmental Data — locality first ─────────────────
# Local BMM150 and Geiger hardware are intentionally NOT used in this build.
#
# Locality hierarchy:
#   1. exact camera-coordinate model/grid when available;
#   2. nearest live ground observatory with recent usable data;
#   3. nearest regional research monitor;
#   4. global/near-Earth space-weather feeds when locality is not meaningful.
#
# Active sources:
#   NOAA/NCEI WMM2025       exact-location geomagnetic field model
#   INTERMAGNET             nearest usable observatory selected from camera GPS
#   USGS Geomagnetism       fallback live observatory data
#   NOAA SWPC OVATION       aurora probability sampled at camera grid location
#   NMDB/Bartol NEWK        nearest-available ground-level cosmic-ray neutron monitoring
#   NOAA SWPC/GOES          solar wind, IMF, Kp, energetic proton/electron flux
#
# CAMERA_LAT/CAMERA_LON may be supplied at launch, or update with:
#   POST /environment/location {"lat":..., "lon":..., "alt_m":...}

ENV_UPDATE_SEC = float(os.environ.get("ENV_UPDATE_SEC", "20"))
ENV_HTTP_TIMEOUT = float(os.environ.get("ENV_HTTP_TIMEOUT", "7"))
OVATION_UPDATE_SEC = float(os.environ.get("OVATION_UPDATE_SEC", "120"))
NMDB_UPDATE_SEC = float(os.environ.get("COSMIC_UPDATE_SEC", os.environ.get("NMDB_UPDATE_SEC", "300")))
LOCATION_FILE = os.environ.get("CAMERA_LOCATION_FILE", "/home/stoopchild11/camera_location.json")

INTERMAGNET_CAP_URL = (
    "https://imag-data.bgs.ac.uk/GIN_V1/GINServices"
    "?Request=GetCapabilities&Format=json"
)
INTERMAGNET_HAPI_DATA = "https://imag-data.bgs.ac.uk/GIN_V1/hapi/data"
USGS_GEOMAG_URL = "https://geomag.usgs.gov/ws/data/"
SWPC_WIND_MAG_URL = "https://services.swpc.noaa.gov/products/summary/solar-wind-mag-field.json"
SWPC_WIND_SPEED_URL = "https://services.swpc.noaa.gov/products/summary/solar-wind-speed.json"
SWPC_KP_URL = "https://services.swpc.noaa.gov/products/noaa-planetary-k-index.json"
SWPC_PROTON_URL = "https://services.swpc.noaa.gov/json/goes/primary/integral-protons-6-hour.json"
SWPC_ELECTRON_URL = "https://services.swpc.noaa.gov/json/goes/primary/integral-electrons-6-hour.json"
SWPC_OVATION_URL = "https://services.swpc.noaa.gov/json/ovation_aurora_latest.json"
NMDB_NEWK_URL = "https://www.nmdb.eu/nest/draw_graph.php"
NMDB_NEWK_LAT = 39.68
NMDB_NEWK_LON = -75.75
OULU_QUERY_URL = "https://cosmicrays.oulu.fi/webform/onlinequery.cgi"
OULU_LAT, OULU_LON = 65.0544, 25.4692
COSMIC_HTTP_TIMEOUT = float(os.environ.get("COSMIC_HTTP_TIMEOUT", "12"))
COSMIC_NEAR_RETRY_SEC = float(os.environ.get("COSMIC_NEAR_RETRY_SEC", "3600"))
COSMIC_MAX_AGE_HOURS = float(os.environ.get("COSMIC_MAX_AGE_HOURS", "8"))
COSMIC_CACHE_FILE = Path(os.path.expanduser(os.environ.get("COSMIC_CACHE_FILE", "~/resonance-cosmic-cache.json")))
_cosmic_near_last_failure = 0.0

_environment_lock = threading.Lock()
_inter_cache_lock = threading.Lock()
_inter_catalog_cache = None
_inter_catalog_cache_time = 0.0
_slow_source_cache = {
    "ovation": {"last": 0.0, "value": None},
    "nmdb": {"last": 0.0, "value": None},
}


def _float_or_none(value):
    try:
        value = float(value)
        if math.isfinite(value):
            return value
    except Exception:
        pass
    return None


def _normalize_lon(lon):
    lon = float(lon)
    while lon > 180.0:
        lon -= 360.0
    while lon < -180.0:
        lon += 360.0
    return lon


def _haversine_km(lat1, lon1, lat2, lon2):
    r = 6371.0088
    p1 = math.radians(float(lat1)); p2 = math.radians(float(lat2))
    dp = math.radians(float(lat2) - float(lat1))
    dl = math.radians(_normalize_lon(float(lon2) - float(lon1)))
    a = math.sin(dp/2.0)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2.0)**2
    return r * 2.0 * math.asin(min(1.0, math.sqrt(max(0.0, a))))


def _load_camera_location():
    lat = _float_or_none(os.environ.get("CAMERA_LAT"))
    lon = _float_or_none(os.environ.get("CAMERA_LON"))
    alt = _float_or_none(os.environ.get("CAMERA_ALT_M", "0")) or 0.0
    if lat is not None and lon is not None:
        return {"lat":lat,"lon":lon,"alt_m":alt,"source":"environment"}
    try:
        with open(LOCATION_FILE, "r", encoding="utf-8") as f:
            saved = json.load(f)
        lat = _float_or_none(saved.get("lat")); lon = _float_or_none(saved.get("lon"))
        alt = _float_or_none(saved.get("alt_m", 0)) or 0.0
        if lat is not None and lon is not None:
            return {"lat":lat,"lon":lon,"alt_m":alt,"source":saved.get("source", "saved"),
                    "accuracy_m":saved.get("accuracy_m"),"updated_at":saved.get("updated_at"),
                    "city":saved.get("city"),"provider":saved.get("provider"),"cached":True}
    except Exception:
        pass
    return {"lat":None,"lon":None,"alt_m":0.0,"source":"unset"}


camera_location = _load_camera_location()
environment_state = {
    "status":"starting", "updated_at":None, "location":dict(camera_location),
    "wmm":{"available":False,"model":"WMM2025","x":None,"y":None,"z":None,"h":None,"f":None,"d":None,"i":None,"error":None},
    "geomag":{"available":False,"provider":"INTERMAGNET","station":None,"station_name":None,"station_lat":None,"station_lon":None,"distance_km":None,"x":None,"y":None,"z":None,"f":None,"dx":None,"dy":None,"dz":None,"df":None,"sample_time":None,"error":None},
    "aurora":{"available":False,"provider":"NOAA SWPC OVATION","probability":None,"grid_lat":None,"grid_lon":None,"grid_distance_km":None,"observation_time":None,"forecast_time":None,"error":None},
    "cosmic_ray":{"available":False,"provider":"nearest-available neutron monitor","station":None,"distance_km":None,"counts_per_sec":None,"relative_percent":None,"sample_time":None,"freshness":"unavailable","error":None},
    "swpc":{"available":False,"solar_wind_speed":None,"bt":None,"bz_gsm":None,"kp":None,"proton_flux_10mev":None,"electron_flux_2mev":None,"sample_time":None,"error":None},
    "errors":[],
}


# ─── v8.4.7 automatic network geolocation ────────────────────────────────────
# IP-based positioning works with any Internet connection, but is *approximate*.
# Optional Google Wi-Fi positioning is enabled only when GEO_WIFI_API_KEY is set.
# It sends nearby access-point MAC addresses to Google; no scan data is shared
# unless the operator explicitly provides that API key.
GEO_AUTO = os.environ.get("GEO_AUTO", "1").strip().lower() not in ("0", "false", "no")
GEO_REFRESH_SEC = max(60.0, float(os.environ.get("GEO_REFRESH_SEC", "600")))
GEO_WIFI_API_KEY = os.environ.get("GEO_WIFI_API_KEY", "").strip()
GEO_WIFI_INTERFACE = os.environ.get("GEO_WIFI_INTERFACE", "wlan0")
GEO_IP_URL = os.environ.get("GEO_IP_URL", "https://ipapi.co/json/")
GEO_DIAGNOSTICS_FILE = os.path.expanduser(os.environ.get("GEO_DIAGNOSTICS_FILE", "~/resonance-geo-status.json"))
_geo_status = {"enabled": GEO_AUTO, "method": None, "last_attempt": None, "last_success": None,
               "error": None, "wifi_enabled": bool(GEO_WIFI_API_KEY)}
_geo_status_lock = threading.Lock()


def _geo_utcnow():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _geo_valid(lat, lon):
    return lat is not None and lon is not None and -90 <= lat <= 90 and -180 <= lon <= 180


def _geo_ip_lookup():
    req = urllib.request.Request(GEO_IP_URL, headers={"User-Agent":"ResonanceCamera/8.4.7"})
    with urllib.request.urlopen(req, timeout=12) as response:
        data = json.load(response)
    lat = _float_or_none(data.get("latitude"))
    lon = _float_or_none(data.get("longitude"))
    if not _geo_valid(lat, lon):
        raise ValueError("IP geolocation returned no valid coordinates")
    return {"lat":lat,"lon":lon,"alt_m":0.0,"source":"auto_ip",
            "accuracy_m":None,"provider":"ipapi.co", "city":data.get("city"),
            "region":data.get("region"),"updated_at":_geo_utcnow(),"cached":False,
            "precision":"regional_approximation"}


def _geo_wifi_lookup():
    # Optional, explicit consent: API key required to transmit scan BSSIDs.
    if not GEO_WIFI_API_KEY:
        raise RuntimeError("Wi-Fi positioning not enabled")
    scan = subprocess.run(["nmcli", "-t", "--escape", "no", "-f", "BSSID,SIGNAL",
                           "device", "wifi", "list", "ifname", GEO_WIFI_INTERFACE],
                          capture_output=True, text=True, timeout=12, check=True)
    aps=[]
    for line in scan.stdout.splitlines():
        match=re.search(r"([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}):([0-9]{1,3})$", line.strip())
        if not match:
            continue
        mac=match.group(1).upper()
        # Locally administered/randomized MACs are unsuitable for geolocation.
        if int(mac[:2],16) & 2:
            continue
        strength=min(100,max(0,int(match.group(2))))
        aps.append({"macAddress":mac,"signalStrength":round(-100 + strength*0.7)})
    aps=list({item["macAddress"]:item for item in aps}.values())[:30]
    if len(aps)<2:
        raise RuntimeError("Not enough usable Wi-Fi access points for positioning")
    url="https://www.googleapis.com/geolocation/v1/geolocate?"+urllib.parse.urlencode({"key":GEO_WIFI_API_KEY})
    body=json.dumps({"considerIp":False,"wifiAccessPoints":aps}).encode("utf-8")
    req=urllib.request.Request(url,data=body,headers={"Content-Type":"application/json",
          "User-Agent":"ResonanceCamera/8.4.7"},method="POST")
    with urllib.request.urlopen(req,timeout=12) as response:
        data=json.load(response)
    loc=data.get("location") or {}
    lat=_float_or_none(loc.get("lat")); lon=_float_or_none(loc.get("lng"))
    if not _geo_valid(lat,lon):
        raise ValueError("Wi-Fi geolocation returned invalid coordinates")
    accuracy=_float_or_none(data.get("accuracy"))
    return {"lat":lat,"lon":lon,"alt_m":0.0,"source":"auto_wifi",
            "accuracy_m":accuracy,"provider":"google_geolocation","city":None,
            "updated_at":_geo_utcnow(),"cached":False,"precision":"estimated"}


def _geo_write_status_file():
    with _geo_status_lock:
        diag=dict(_geo_status)
    with _environment_lock:
        diag["location"]=dict(environment_state.get("location",{}))
        diag["environment"] = json.loads(json.dumps(environment_state))
    diag["diagnostic_time"]=_geo_utcnow()
    try:
        path=Path(GEO_DIAGNOSTICS_FILE)
        path.parent.mkdir(parents=True,exist_ok=True)
        temp=path.with_name(path.name+".tmp")
        temp.write_text(json.dumps(diag,indent=2,default=str)+"\n",encoding="utf-8")
        os.replace(temp,path)
        return True
    except Exception as e:
        print(f"GEO diagnostic write failed: {e}",flush=True)
        return False


def _geo_export_signal(_signum, _frame):
    # Signal handler performs no disk IO; work happens on diagnostic thread.
    _geo_export_requested.set()


_geo_export_requested=threading.Event()


def _geo_diagnostics_loop():
    while True:
        _geo_export_requested.wait()
        _geo_export_requested.clear()
        if _geo_write_status_file():
            print(f"GEO diagnostic exported: {GEO_DIAGNOSTICS_FILE}",flush=True)


def _geo_save_location(loc):
    try:
        path=Path(LOCATION_FILE)
        path.parent.mkdir(parents=True, exist_ok=True)
        temp=path.with_name(path.name+".tmp")
        temp.write_text(json.dumps(loc,indent=2)+"\n",encoding="utf-8")
        os.replace(temp,path)
    except Exception as e:
        print(f"GEO location cache write failed: {e}",flush=True)


def _geo_poll_loop():
    global camera_location
    if not GEO_AUTO:
        print("GEO: automatic network positioning disabled",flush=True)
        return
    while True:
        with _environment_lock:
            existing=dict(environment_state.get("location",{}))
        # Explicit user coordinates always win; cached automatic readings don't.
        if existing.get("source") not in ("environment", "runtime", "manual", "saved"):
            errors=[]
            with _geo_status_lock:
                _geo_status["last_attempt"]=_geo_utcnow()
            candidate=None
            if GEO_WIFI_API_KEY:
                try:
                    candidate=_geo_wifi_lookup()
                except Exception as e:
                    errors.append("Wi-Fi: "+str(e))
            if candidate is None:
                try:
                    candidate=_geo_ip_lookup()
                except Exception as e:
                    errors.append("IP: "+str(e))
            if candidate is not None:
                with _environment_lock:
                    current=dict(environment_state.get("location",{}))
                    if current.get("source") not in ("environment", "runtime", "manual", "saved"):
                        changed=(current.get("lat") != candidate["lat"] or
                                 current.get("lon") != candidate["lon"] or
                                 current.get("source") != candidate["source"])
                        camera_location=dict(candidate)
                        environment_state["location"]=dict(candidate)
                        if changed:
                            for entry in _slow_source_cache.values():
                                entry["last"]=0.0
                                entry["value"]=None
                _geo_save_location(candidate)
                with _geo_status_lock:
                    _geo_status.update({"method":candidate["source"],"last_success":_geo_utcnow(),
                                        "error":"; ".join(errors) if errors else None})
                print(f"GEO: {candidate['source']} {candidate['lat']:.5f}, {candidate['lon']:.5f}"
                      f" ({candidate.get('city') or 'unknown city'}; approximate)",flush=True)
            else:
                with _geo_status_lock:
                    _geo_status["error"]="; ".join(errors)
                print("GEO: positioning unavailable; "+"; ".join(errors),flush=True)
        else:
            with _geo_status_lock:
                _geo_status["method"]="manual_override"
        _geo_write_status_file()
        time.sleep(GEO_REFRESH_SEC)


def _http_text(url):
    req = urllib.request.Request(url, headers={"User-Agent":"AntiSurveillanceCamera/2.0 (+RaspberryPi)"})
    with urllib.request.urlopen(req, timeout=ENV_HTTP_TIMEOUT) as response:
        return response.read().decode("utf-8", "replace")


def _http_json(url):
    return json.loads(_http_text(url))


def _decimal_year(dt):
    dt = dt.astimezone(timezone.utc)
    start = datetime(dt.year,1,1,tzinfo=timezone.utc)
    end = datetime(dt.year+1,1,1,tzinfo=timezone.utc)
    return dt.year + (dt-start).total_seconds()/(end-start).total_seconds()


def _calculate_wmm(location):
    lat = _float_or_none(location.get("lat")); lon = _float_or_none(location.get("lon"))
    alt = _float_or_none(location.get("alt_m")) or 0.0
    base = {"available":False,"model":"WMM2025","x":None,"y":None,"z":None,"h":None,"f":None,"d":None,"i":None,"error":None}
    if lat is None or lon is None:
        base["error"] = "camera latitude/longitude not set"; return base
    try:
        from pygeomag import GeoMag
        gm = GeoMag()
        r = gm.calculate(glat=lat, glon=lon, alt=alt/1000.0, time=_decimal_year(datetime.now(timezone.utc)))
        base.update({"available":True,"x":round(float(r.x),2),"y":round(float(r.y),2),"z":round(float(r.z),2),"h":round(float(r.h),2),"f":round(float(r.f),2),"d":round(float(r.d),3),"i":round(float(r.i),3)})
        return base
    except Exception as e:
        base["error"] = f"WMM unavailable: {e}"; return base


def _recursive_observatories(obj, found):
    if isinstance(obj, dict):
        lower = {str(k).lower().replace("_",""):v for k,v in obj.items()}
        code = lower.get("iagacode") or lower.get("observatoryiagacode") or lower.get("code") or lower.get("id")
        lat = lower.get("latitude") or lower.get("geographiclatitude") or lower.get("lat")
        lon = lower.get("longitude") or lower.get("geographiclongitude") or lower.get("lon") or lower.get("lng")
        name = lower.get("name") or lower.get("observatoryname") or code
        latf = _float_or_none(lat); lonf = _float_or_none(lon)
        codes = str(code).strip().upper() if code is not None else ""
        if len(codes)==3 and codes.isalpha() and latf is not None and lonf is not None:
            found[codes] = {"code":codes,"name":str(name).strip() if name else codes,"lat":latf,"lon":_normalize_lon(lonf)}
        for v in obj.values():
            if isinstance(v,(dict,list)): _recursive_observatories(v, found)
    elif isinstance(obj,list):
        for v in obj: _recursive_observatories(v, found)


def _intermagnet_catalog():
    global _inter_catalog_cache, _inter_catalog_cache_time
    with _inter_cache_lock:
        if _inter_catalog_cache is not None and time.monotonic()-_inter_catalog_cache_time < 21600:
            return list(_inter_catalog_cache)
    stations = {}
    try:
        _recursive_observatories(_http_json(INTERMAGNET_CAP_URL), stations)
    except Exception:
        pass
    for s in [
        {"code":"OTT","name":"Ottawa, Canada","lat":45.403,"lon":-75.552},
        {"code":"FRD","name":"Fredericksburg, USA","lat":38.2047,"lon":-77.3729},
        {"code":"STJ","name":"St. John's, Canada","lat":47.595,"lon":-52.677},
        {"code":"BOU","name":"Boulder, USA","lat":40.1375,"lon":-105.2372},
        {"code":"SJG","name":"San Juan, Puerto Rico","lat":18.111,"lon":-66.15},
    ]:
        stations.setdefault(s["code"],s)
    out = list(stations.values())
    with _inter_cache_lock:
        _inter_catalog_cache = list(out); _inter_catalog_cache_time = time.monotonic()
    return out


def _parse_hapi_xyzf(payload):
    rows = payload.get("data",[]) if isinstance(payload,dict) else []
    samples=[]
    for row in rows:
        if not isinstance(row,list) or len(row)<2: continue
        vals=[_float_or_none(v) for v in row[1:5]]
        if any(v is not None for v in vals): samples.append((row[0],vals))
    if not samples: return None
    out={"sample_time":samples[-1][0]}
    for idx,label in enumerate(("x","y","z","f")):
        vals=[v[idx] for _t,v in samples if idx < len(v) and v[idx] is not None]
        if vals:
            latest=vals[-1]; baseline=statistics.median(vals[-min(20,len(vals)):])
            out[label]=round(latest,2); out["d"+label]=round(latest-baseline,2)
        else:
            out[label]=None; out["d"+label]=None
    return out


def _fetch_intermagnet_station(station):
    end=datetime.now(timezone.utc); start=end-timedelta(minutes=45)
    dataset=f"{station['code'].lower()}/best-avail/PT1M/xyzf"
    q={"id":dataset,"time.min":start.strftime("%Y-%m-%dT%H:%M:%SZ"),"time.max":end.strftime("%Y-%m-%dT%H:%M:%SZ"),"format":"json"}
    return _parse_hapi_xyzf(_http_json(INTERMAGNET_HAPI_DATA+"?"+urllib.parse.urlencode(q)))


def _collect_usgs_series(payload):
    found={}; times=[]
    def walk(obj):
        nonlocal times
        if isinstance(obj,dict):
            if isinstance(obj.get("times"),list) and len(obj["times"])>len(times): times=obj["times"]
            meta=obj.get("metadata") if isinstance(obj.get("metadata"),dict) else {}
            el=obj.get("element") or meta.get("element") or meta.get("channel")
            vals=obj.get("values")
            if el and isinstance(vals,list): found[str(el).upper()]=vals
            for v in obj.values():
                if isinstance(v,(dict,list)): walk(v)
        elif isinstance(obj,list):
            for v in obj: walk(v)
    walk(payload); return found,times


def _finite_series(values):
    out=[]
    for value in values if isinstance(values,list) else []:
        if isinstance(value,dict): value=value.get("value")
        v=_float_or_none(value)
        if v is not None and abs(v)<1e10: out.append(v)
    return out


def _fetch_usgs_frd(location):
    lat0,lon0=38.2047,-77.3729
    end=datetime.now(timezone.utc); start=end-timedelta(minutes=30)
    q={"id":"FRD","format":"json","elements":"X,Y,Z,F","sampling_period":"60","type":"adjusted","starttime":start.strftime("%Y-%m-%dT%H:%M:%SZ"),"endtime":end.strftime("%Y-%m-%dT%H:%M:%SZ")}
    base={"available":False,"provider":"USGS fallback","station":"FRD","station_name":"Fredericksburg","station_lat":lat0,"station_lon":lon0,"distance_km":None,"x":None,"y":None,"z":None,"f":None,"dx":None,"dy":None,"dz":None,"df":None,"sample_time":None,"error":None}
    try:
        lat=_float_or_none(location.get("lat")); lon=_float_or_none(location.get("lon"))
        if lat is not None and lon is not None: base["distance_km"]=round(_haversine_km(lat,lon,lat0,lon0),1)
        series,times=_collect_usgs_series(_http_json(USGS_GEOMAG_URL+"?"+urllib.parse.urlencode(q)))
        base["sample_time"]=times[-1] if times else None; usable=0
        for el in ("X","Y","Z","F"):
            vals=_finite_series(series.get(el,[]))
            if vals:
                latest=vals[-1]; median=statistics.median(vals[-min(20,len(vals)):])
                base[el.lower()]=round(latest,2); base["d"+el.lower()]=round(latest-median,2); usable+=1
        base["available"]=usable>0
        if not base["available"]: base["error"]="no usable USGS samples"
    except Exception as e: base["error"]=str(e)
    return base


def _nearest_geomag(location):
    lat=_float_or_none(location.get("lat")); lon=_float_or_none(location.get("lon"))
    if lat is None or lon is None:
        return {"available":False,"provider":"INTERMAGNET","station":None,"station_name":None,"station_lat":None,"station_lon":None,"distance_km":None,"x":None,"y":None,"z":None,"f":None,"dx":None,"dy":None,"dz":None,"df":None,"sample_time":None,"error":"camera location not set"}
    candidates=sorted(_intermagnet_catalog(), key=lambda s:_haversine_km(lat,lon,s["lat"],s["lon"]))
    errors=[]
    for station in candidates[:12]:
        try:
            vals=_fetch_intermagnet_station(station)
            if vals and any(vals.get(k) is not None for k in ("x","y","z","f")):
                return {"available":True,"provider":"INTERMAGNET","station":station["code"],"station_name":station["name"],"station_lat":station["lat"],"station_lon":station["lon"],"distance_km":round(_haversine_km(lat,lon,station["lat"],station["lon"]),1),**vals,"error":None}
        except Exception as e: errors.append(f"{station['code']}: {e}")
    fallback=_fetch_usgs_frd(location)
    if fallback.get("available"): return fallback
    return {"available":False,"provider":"INTERMAGNET","station":None,"station_name":None,"station_lat":None,"station_lon":None,"distance_km":None,"x":None,"y":None,"z":None,"f":None,"dx":None,"dy":None,"dz":None,"df":None,"sample_time":None,"error":"; ".join(errors[-3:]) or fallback.get("error")}


def _latest_flux(records,wanted_energy):
    wanted=wanted_energy.replace(" ","").lower()
    for row in reversed(records if isinstance(records,list) else []):
        if isinstance(row,dict) and wanted in str(row.get("energy","")).replace(" ","").lower():
            v=_float_or_none(row.get("flux"))
            if v is not None: return v,row.get("time_tag")
    return None,None


def _fetch_swpc():
    out={"available":False,"solar_wind_speed":None,"bt":None,"bz_gsm":None,"kp":None,"proton_flux_10mev":None,"electron_flux_2mev":None,"sample_time":None,"error":None}
    errors=[]; times=[]
    for name,url in (("mag",SWPC_WIND_MAG_URL),("speed",SWPC_WIND_SPEED_URL),("kp",SWPC_KP_URL)):
        try:
            data=_http_json(url); row=data[-1] if isinstance(data,list) and data else {}
            if name=="mag": out["bt"]=_float_or_none(row.get("bt")); out["bz_gsm"]=_float_or_none(row.get("bz_gsm"))
            elif name=="speed": out["solar_wind_speed"]=_float_or_none(row.get("proton_speed"))
            else: out["kp"]=_float_or_none(row.get("Kp"))
            if row.get("time_tag"): times.append(row["time_tag"])
        except Exception as e: errors.append(f"{name}: {e}")
    try:
        v,t=_latest_flux(_http_json(SWPC_PROTON_URL),">=10 MeV"); out["proton_flux_10mev"]=v; times += [t] if t else []
    except Exception as e: errors.append(f"protons: {e}")
    try:
        data=_http_json(SWPC_ELECTRON_URL); v,t=_latest_flux(data,">=2 MeV")
        if v is None: v,t=_latest_flux(data,">2 MeV")
        out["electron_flux_2mev"]=v; times += [t] if t else []
    except Exception as e: errors.append(f"electrons: {e}")
    out["sample_time"]=times[-1] if times else None
    out["available"]=any(out[k] is not None for k in ("solar_wind_speed","bt","bz_gsm","kp","proton_flux_10mev","electron_flux_2mev"))
    out["error"]="; ".join(errors) if errors else None
    return out


def _fetch_ovation(location):
    lat=_float_or_none(location.get("lat")); lon=_float_or_none(location.get("lon"))
    base={"available":False,"provider":"NOAA SWPC OVATION","probability":None,"grid_lat":None,"grid_lon":None,"grid_distance_km":None,"observation_time":None,"forecast_time":None,"error":None}
    if lat is None or lon is None: base["error"]="camera location not set"; return base
    try:
        payload=_http_json(SWPC_OVATION_URL); best=None; metric=1e99
        for row in payload.get("coordinates",[]) if isinstance(payload,dict) else []:
            if not isinstance(row,list) or len(row)<3: continue
            glon=_float_or_none(row[0]); glat=_float_or_none(row[1]); prob=_float_or_none(row[2])
            if glon is None or glat is None or prob is None: continue
            glon=_normalize_lon(glon); dlat=glat-lat; dlon=_normalize_lon(glon-lon)*math.cos(math.radians(lat)); m=dlat*dlat+dlon*dlon
            if m<metric: metric=m; best=(glat,glon,prob)
        if best is None: raise RuntimeError("no usable OVATION grid")
        glat,glon,prob=best; base.update({"available":True,"probability":round(prob,3),"grid_lat":glat,"grid_lon":glon,"grid_distance_km":round(_haversine_km(lat,lon,glat,glon),1),"observation_time":payload.get("Observation Time"),"forecast_time":payload.get("Forecast Time")})
    except Exception as e: base["error"]=str(e)
    return base


def _fetch_nmdb_newk(location):
    lat=_float_or_none(location.get("lat")); lon=_float_or_none(location.get("lon"))
    dist=round(_haversine_km(lat,lon,NMDB_NEWK_LAT,NMDB_NEWK_LON),1) if lat is not None and lon is not None else None
    base={"available":False,"provider":"NMDB / Bartol NEWK","station":"NEWK","distance_km":dist,"counts_per_sec":None,"relative_percent":None,"sample_time":None,"error":None}
    q=[("formchk","1"),("stations[]","NEWK"),("output","ascii"),("tabchoice","revori"),("dtype","corr_for_efficiency"),("date_choice","last"),("last_days","60"),("last_label","mins_label"),("tresolution","0"),("force","1"),("yunits","0")]
    try:
        raw=_http_text(NMDB_NEWK_URL+"?"+urllib.parse.urlencode(q)); values=[]
        for line in raw.splitlines():
            if not re.search(r"\d{4}-\d{2}-\d{2}",line): continue
            nums=[_float_or_none(p.strip()) for p in line.split(";")[1:]]; nums=[v for v in nums if v is not None]
            if nums:
                m=re.search(r"\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}(?::\d{2})?",line); values.append((m.group(0) if m else None,nums[-1]))
        if not values: raise RuntimeError("NEWK returned no parseable data")
        ts,latest=values[-1]; baseline=statistics.median([v for _t,v in values[-min(30,len(values)):]])
        rel=((latest/baseline)-1.0)*100.0 if baseline else None
        base.update({"available":True,"counts_per_sec":round(latest,4),"relative_percent":round(rel,3) if rel is not None else None,"sample_time":ts})
    except Exception as e: base["error"]=str(e)
    return base



# v8.4.8: an accessible neutron-monitor observation is a regional proxy, never
# a location-exact count. Observations retain station, distance and timestamp.
def _cosmic_iso(text):
    if not text: return None
    try: value=datetime.fromisoformat(str(text).strip().replace("Z", "+00:00"))
    except (ValueError, TypeError): return None
    if value.tzinfo is None: value=value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def _cosmic_station_distance(location, station):
    lat=_float_or_none(location.get("lat")); lon=_float_or_none(location.get("lon"))
    if lat is None or lon is None: return None
    slat,slon=(NMDB_NEWK_LAT,NMDB_NEWK_LON) if station=="NEWK" else (OULU_LAT,OULU_LON)
    return round(_haversine_km(lat,lon,slat,slon),1)


def _cosmic_cache_read():
    try:
        d=json.loads(COSMIC_CACHE_FILE.read_text(encoding="utf-8"))
        if isinstance(d,dict) and d.get("sample_time") and _float_or_none(d.get("counts_per_sec")) is not None:
            return d
    except Exception: pass
    return None


def _cosmic_cache_write(result):
    try:
        COSMIC_CACHE_FILE.parent.mkdir(parents=True,exist_ok=True)
        temporary=COSMIC_CACHE_FILE.with_suffix(".tmp")
        temporary.write_text(json.dumps(result,indent=2),encoding="utf-8")
        temporary.replace(COSMIC_CACHE_FILE)
    except Exception as exc:
        print(f"COSMIC: cache write failed: {exc}",flush=True)


def _cosmic_validate(result, location):
    moment=_cosmic_iso(result.get("sample_time"))
    if not moment: raise ValueError("station returned no valid observation timestamp")
    age=(datetime.now(timezone.utc)-moment).total_seconds()/3600.0
    if age < -0.25 or age > COSMIC_MAX_AGE_HOURS:
        raise ValueError(f"station observation too old or future-dated ({age:.1f} hours)")
    count=_float_or_none(result.get("counts_per_sec"))
    if count is None or count<=0: raise ValueError("station returned no positive count rate")
    result["distance_km"]=_cosmic_station_distance(location,result["station"])
    result["age_hours"]=round(age,2)
    result["freshness"]="live"
    result["available"]=True
    result["error"]=None
    return result


def _fetch_oulu_cosmic(location):
    # Confirmed response: data[] rows include 'time:', 'corr:' counts/min,
    # 'uncorr:' counts/min, 'pressure:' mbar. Date and month MUST be zero-padded.
    end=datetime.now(timezone.utc)-timedelta(hours=2)
    start=end-timedelta(hours=24)
    params={"station":"OULU", "startday":f"{start.day:02d}",
            "startmonth":f"{start.month:02d}","startyear":str(start.year),
            "starttime":f"{start.hour:02d}:00",
            "endday":f"{end.day:02d}","endmonth":f"{end.month:02d}",
            "endyear":str(end.year),"endtime":f"{end.hour:02d}:00",
            "resolution":"5","outputmode":"json"}
    url=OULU_QUERY_URL+"?"+urllib.parse.urlencode(params)
    request=urllib.request.Request(url,headers={"User-Agent":"ResonanceCamera/8.4.8"})
    with urllib.request.urlopen(request,timeout=COSMIC_HTTP_TIMEOUT) as response:
        raw=response.read(1500000)
    data=json.loads(raw)
    if not isinstance(data,dict) or not isinstance(data.get("data"),list):
        raise ValueError("Oulu JSON missing measurement data")
    values=[]
    for row in data["data"]:
        if not isinstance(row,dict): continue
        moment=_cosmic_iso(row.get("time:") or row.get("time"))
        cpm=_float_or_none(row.get("corr:"))
        if moment is None or cpm is None or cpm<=0: continue
        values.append((moment,cpm,row))
    if not values: raise ValueError("Oulu returned no usable corrected neutron counts")
    values.sort(key=lambda x:x[0])
    t,cpm,row=values[-1]
    # Local 24h median is specific to OULU, avoiding inter-station calibration jumps.
    median=statistics.median(x[1] for x in values)
    rel=100.0*(cpm/median-1.0) if median>0 else None
    result={"available":True,"provider":"University of Oulu neutron monitor",
            "station":"OULU","station_name":"Oulu, Finland",
            "distance_km":_cosmic_station_distance(location,"OULU"),
            "counts_per_sec":round(cpm/60.0,4),
            "corrected_counts_per_min":round(cpm,3),
            "uncorrected_counts_per_min":_float_or_none(row.get("uncorr:")),
            "pressure_mbar":_float_or_none(row.get("pressure:")),
            "relative_percent":round(rel,3) if rel is not None else None,
            "sample_time":t.isoformat(timespec="seconds").replace("+00:00","Z"),
            "resolution_minutes":5,"error":None}
    return _cosmic_validate(result,location)


def _fetch_cosmic_nearest_available(location):
    # NEWK is nearer to northeastern USA, but NMDB has been unreachable.
    # Circuit breaker prevents a failed site from blocking every refresh.
    global _cosmic_near_last_failure
    errors=[]
    candidates=[("NEWK",_cosmic_station_distance(location,"NEWK")),
                ("OULU",_cosmic_station_distance(location,"OULU"))]
    candidates.sort(key=lambda v: float('inf') if v[1] is None else v[1])
    for station,_ in candidates:
        if station=="NEWK":
            if time.monotonic()-_cosmic_near_last_failure<COSMIC_NEAR_RETRY_SEC:
                continue
            try:
                # Existing NMDB parser returns per-second data; validate timestamp.
                result=_fetch_nmdb_newk(location)
                if not result.get("available"): raise RuntimeError(result.get("error") or "NEWK unavailable")
                # The old NMDB timestamps can have a space separator and no timezone.
                sample=result.get("sample_time")
                if sample and len(sample)>=16 and not str(sample).endswith("Z"):
                    result["sample_time"]=str(sample).replace(" ","T")+"Z"
                result["station_name"]="Newark/Swarthmore, Delaware"
                result["provider"]="NMDB / Bartol NEWK"
                result=_cosmic_validate(result,location)
                _cosmic_cache_write(result)
                return result
            except Exception as exc:
                errors.append("NEWK: "+str(exc))
                _cosmic_near_last_failure=time.monotonic()
        else:
            try:
                result=_fetch_oulu_cosmic(location)
                _cosmic_cache_write(result)
                return result
            except Exception as exc:
                errors.append("OULU: "+str(exc))
    cached=_cosmic_cache_read()
    if cached:
        cached=dict(cached)
        cached["available"]=True  # measured data, but not current
        cached["freshness"]="cached"
        cached["distance_km"]=_cosmic_station_distance(location,cached.get("station","OULU"))
        instant=_cosmic_iso(cached.get("sample_time"))
        cached["age_hours"]=round((datetime.now(timezone.utc)-instant).total_seconds()/3600,2) if instant else None
        cached["error"]="; ".join(errors) if errors else "No fresh source available"
        return cached
    return {"available":False,"provider":"nearest-available neutron monitor",
            "station":None,"distance_km":None,"counts_per_sec":None,
            "relative_percent":None,"sample_time":None,"freshness":"unavailable",
            "error":"; ".join(errors) or "No neutron-monitor source accessible"}

def _slow_cached(name,interval,fetcher,location):
    now=time.monotonic(); item=_slow_source_cache[name]
    if item["value"] is None or now-item["last"]>=interval:
        item["value"]=fetcher(location); item["last"]=now
    return dict(item["value"])


def _environment_snapshot():
    with _environment_lock: return json.loads(json.dumps(environment_state))


def _update_environment_once():
    with _environment_lock: location=dict(environment_state.get("location",camera_location))
    wmm=_calculate_wmm(location); geomag=_nearest_geomag(location); swpc=_fetch_swpc()
    aur=_slow_cached("ovation",OVATION_UPDATE_SEC,_fetch_ovation,location)
    cosmic=_slow_cached("nmdb",NMDB_UPDATE_SEC,_fetch_cosmic_nearest_available,location)
    sources=(wmm,geomag,swpc,aur,cosmic); errors=[s.get("error") for s in sources if s.get("error")]
    with _environment_lock:
        environment_state.update({"wmm":wmm,"geomag":geomag,"swpc":swpc,"aurora":aur,"cosmic_ray":cosmic,"updated_at":datetime.now(timezone.utc).isoformat(timespec="seconds"),"errors":errors,"status":"live" if any(s.get("available") for s in sources) else "offline"})


def environment_poll_loop():
    while True:
        try: _update_environment_once()
        except Exception as e:
            with _environment_lock: environment_state["status"]="error"; environment_state["errors"]=[str(e)]
        time.sleep(max(5.0,ENV_UPDATE_SEC))

threading.Thread(target=environment_poll_loop,daemon=True,name="nearest-environment-data").start()

# ─── Camera Controls State ────────────────────────────────────────────────────
cam_controls = {
    'auto_exposure': True,
    'exposure_time': 20000,   # microseconds
    'analogue_gain': 1.0,     # 1.0–16.0
    'ev_compensation': 0.0,   # -4.0 to +4.0 (auto only)
    'brightness': 0.0,        # -1.0 to 1.0
    'contrast': 1.0,          # 0.0 to 2.0
    'saturation': 1.0,        # 0.0 to 2.0
}
_cam_lock = threading.Lock()

def apply_cam_controls():
    with _cam_lock:
        ctrl = dict(cam_controls)
    controls = {
        'Brightness': float(ctrl['brightness']),
        'Contrast':   float(ctrl['contrast']),
        'Saturation': float(ctrl['saturation']),
    }
    if ctrl['auto_exposure']:
        controls['AeEnable'] = True
        controls['ExposureValue'] = float(ctrl['ev_compensation'])
        # Prevent auto exposure from silently lowering the sensor below 30 fps.
        controls['FrameDurationLimits'] = (16666, PREVIEW_FRAME_MAX_US)
    else:
        exposure_us = int(ctrl['exposure_time'])
        controls['AeEnable'] = False
        controls['ExposureTime'] = exposure_us
        controls['AnalogueGain'] = float(ctrl['analogue_gain'])
        # Manual exposures longer than ~33 ms intentionally reduce live FPS.
        controls['FrameDurationLimits'] = (16666, max(PREVIEW_FRAME_MAX_US, exposure_us + 1000))
    try:
        picam2.set_controls(controls)
    except Exception as e:
        print(f"Camera control error: {e}")

# ─── 4D Hyperspace Settings ───────────────────────────────────────────────────
hyperspace_settings = {
    'submode':   'tesseract',
    'rot_speed': 1.0,
    'w_depth':   2.5,
}

# ─── 4D Geometry (precomputed at startup) ─────────────────────────────────────

# Tesseract: 16 vertices at (±1)^4, 32 edges
_TESS_VERTS = np.array(
    [[x, y, z, w] for x in (-1., 1.) for y in (-1., 1.)
     for z in (-1., 1.) for w in (-1., 1.)], dtype=np.float32)
_TESS_EDGES = [(i, j) for i in range(16) for j in range(i+1, 16)
               if bin(i ^ j).count('1') == 1]

# 16-cell: 8 vertices, 24 edges
_CELL16_VERTS = np.array([
    [1,0,0,0],[-1,0,0,0],[0,1,0,0],[0,-1,0,0],
    [0,0,1,0],[0,0,-1,0],[0,0,0,1],[0,0,0,-1]], dtype=np.float32)
_CELL16_EDGES = [(i, j) for i in range(8) for j in range(i+1, 8)
                 if i // 2 != j // 2]

# Clifford torus: (cosθ, sinθ, cosφ, sinφ)/√2
_N_CLIFF = 32
_cliff_t = np.linspace(0, 2*np.pi, _N_CLIFF, endpoint=False)
_cliff_p = np.linspace(0, 2*np.pi, _N_CLIFF, endpoint=False)
_CT, _CP = np.meshgrid(_cliff_t, _cliff_p)
_CLIFF_VERTS = (np.stack([
    np.cos(_CT.ravel()), np.sin(_CT.ravel()),
    np.cos(_CP.ravel()), np.sin(_CP.ravel())
], axis=1) / np.sqrt(2)).astype(np.float32)
_CLIFF_PHI_FLAT = _CP.ravel().astype(np.float32)

# 4D Lissajous parameter samples
_LISS_T = np.linspace(0, 2*np.pi, 512, dtype=np.float32)

def rot4d(plane, angle):
    """4D rotation matrix in the given (i,j) plane."""
    R = np.eye(4, dtype=np.float32)
    i, j = plane
    c, s = float(np.cos(angle)), float(np.sin(angle))
    R[i,i]=c; R[i,j]=-s; R[j,i]=s; R[j,j]=c
    return R

def proj4d(verts, d4=2.5, d3=3.0):
    """Double perspective: 4D → 3D → 2D. Returns (points2d, depth)."""
    w = verts[:, 3]
    f4 = d4 / np.clip(d4 - w, 0.01, None)
    v3 = verts[:, :3] * f4[:, None]
    z = v3[:, 2]
    f3 = d3 / np.clip(d3 - z, 0.01, None)
    v2 = v3[:, :2] * f3[:, None]
    return v2, f4

# Normalised coordinate grids for image warping.  The earlier build used a
# fixed 480×640 grid, which broke when rendering high-resolution stills.  Cache
# grids by output size so the same effects work for both the 720p phone preview
# and the 2K/QHD saved image.
_grid_cache = {}
_grid_cache_lock = threading.Lock()

def _normalized_grids(h, w):
    key = (int(h), int(w))
    with _grid_cache_lock:
        cached = _grid_cache.get(key)
        if cached is not None:
            return cached
        yn, xn = np.mgrid[0:h, 0:w].astype(np.float32)
        xn = (xn / max(1, w)) * 2.0 - 1.0
        yn = (yn / max(1, h)) * 2.0 - 1.0
        _grid_cache[key] = (xn, yn)
        # Avoid unbounded growth if custom resolutions are tested repeatedly.
        if len(_grid_cache) > 6:
            oldest = next(iter(_grid_cache))
            if oldest != key:
                _grid_cache.pop(oldest, None)
        return xn, yn

def _warp_image(img_arr, dx, dy, border=cv2.BORDER_REFLECT):
    """Apply a pixel displacement map to an image array at any resolution."""
    h, w = img_arr.shape[:2]
    xn, yn = _normalized_grids(h, w)
    map_x = ((xn + 1) / 2 * w + dx).astype(np.float32)
    map_y = ((yn + 1) / 2 * h + dy).astype(np.float32)
    return cv2.remap(img_arr, map_x, map_y, cv2.INTER_LINEAR, borderMode=border)


def _warp_channel(channel_arr, dx, dy, border=cv2.BORDER_REFLECT):
    """Warp a single image channel and always return a 2D array.

    cv2.remap collapses H×W×1 arrays to H×W, so the 4D chromatic split must
    treat channels as 2D.  This prevents hyperspace mode from crashing the
    stream with "too many indices" errors.
    """
    warped = _warp_image(channel_arr, dx, dy, border=border)
    if warped.ndim == 3:
        warped = warped[:, :, 0]
    return warped

def scale_boxes(people, from_size, to_size):
    """Scale detector boxes between the lores preview and the main still."""
    from_w, from_h = from_size
    to_w, to_h = to_size
    sx = float(to_w) / max(1.0, float(from_w))
    sy = float(to_h) / max(1.0, float(from_h))
    return [(x1*sx, y1*sy, x2*sx, y2*sy, conf)
            for (x1, y1, x2, y2, conf) in people]


def rotate_boxes_90ccw(people, orig_w):
    """Transform boxes to match PIL's 90° counter-clockwise rotation."""
    return [(y1, orig_w-x2, y2, orig_w-x1, conf)
            for (x1, y1, x2, y2, conf) in people]

app = Flask(__name__)

# ─── Camera ───────────────────────────────────────────────────────────────────
# Two simultaneous streams keep the phone interface responsive while preserving
# a much larger sensor output for saved photographs.  Because the camera is
# mounted in portrait orientation, the raw 2560×1440 still becomes 1440×2560
# after the script's 90° rotation.
# The phone stream targets 30 fps at a modest 640x360 resolution. Saved captures
# still use STILL_SIZE. Detection, clean-plate alignment, scene learning, and
# sensor reads run at lower independent rates so they do not stall the display.
PREVIEW_SIZE = (
    int(os.environ.get("PREVIEW_WIDTH", "320")),
    int(os.environ.get("PREVIEW_HEIGHT", "240")),
)
STILL_SIZE = (
    int(os.environ.get("STILL_WIDTH", "2560")),
    int(os.environ.get("STILL_HEIGHT", "1440")),
)
STILL_JPEG_QUALITY = int(np.clip(int(os.environ.get("STILL_JPEG_QUALITY", "96")), 80, 100))
PREVIEW_JPEG_QUALITY = int(np.clip(int(os.environ.get("PREVIEW_JPEG_QUALITY", "55")), 35, 85))
PREVIEW_FPS = float(np.clip(float(os.environ.get("PREVIEW_FPS", "20")), 3.0, 30.0))
PREVIEW_EFFECT_FPS = float(np.clip(float(os.environ.get("PREVIEW_EFFECT_FPS", "8")), 1.0, 20.0))
# RAW sends fresh camera frames to TFT without waiting for expensive FX rendering.
# Set LCD_SOURCE=fx to restore artistically processed TFT output.
LCD_SOURCE = os.environ.get("LCD_SOURCE", "fx").strip().lower()
if LCD_SOURCE not in ("raw", "fx"):
    raise ValueError("LCD_SOURCE must be raw or fx")
# Local-first mode skips web JPEG production and does not launch Flask.
LOCAL_ONLY = os.environ.get("LOCAL_ONLY", "1") == "1"
PHOTO_REVIEW_SEC = float(os.environ.get("PHOTO_REVIEW_SEC", "12"))  # 0 = until joystick click
LCD_FX_FPS = float(os.environ.get("LCD_FX_FPS", "8"))
PREVIEW_MODE = os.environ.get("PREVIEW_MODE", "fast").strip().lower()
if PREVIEW_MODE not in ("fast", "fx"):
    raise ValueError("PREVIEW_MODE must be fast or fx")
# Simplify OpenCV scheduling on the Pi 4; 20 competing OpenCV workers can
# make each request unpredictably slow when YOLO and SPI are also running.
cv2.setNumThreads(max(1, int(os.environ.get("CV_THREADS", "1"))))
ENABLE_DETECTION = os.environ.get("ENABLE_DETECTION", "0") == "1"
PREVIEW_FRAME_MAX_US = int(np.clip(int(os.environ.get("PREVIEW_FRAME_MAX_US", "33333")), 20000, 100000))
DETECT_FPS = float(np.clip(float(os.environ.get("DETECT_FPS", "5")), 1.0, 10.0))
BACKGROUND_LEARN_EVERY = int(np.clip(int(os.environ.get("BACKGROUND_LEARN_EVERY", "10")), 1, 30))
SCENE_SCAN_EVERY = int(np.clip(int(os.environ.get("SCENE_SCAN_EVERY", "3")), 1, 12))
SCENE_MAP_HIRES_SAMPLES = int(np.clip(int(os.environ.get("SCENE_MAP_HIRES_SAMPLES", "3")), 1, 8))

if LCD_ONLY_TEST:
    print("\n" + "=" * 60)
    print(" LCD-ONLY DIAGNOSTIC MODE")
    print(" Camera, YOLO, Flask, joystick, and preview processing are NOT started.")
    print(f" Logical UI canvas: {_lcd_width}x{_lcd_height}; native TFT: {_lcd_native_width}x{_lcd_native_height}")
    print("=" * 60)
    if LCD_BENCHMARK and HAS_LCD:
        # Measure real write+ACK time through the selected SPI backend.
        # A benchmark is only successful if the panel visibly changes colors.
        n = 12
        colors = [(25, 40, 65), (180, 80, 50), (50, 160, 170)]
        started = time.monotonic()
        ok = True
        for i in range(n):
            ok = _lcd_write_solid(colors[i % len(colors)])
            if not ok:
                break
        elapsed = max(1e-6, time.monotonic() - started)
        print(f"LCD benchmark ({LCD_BACKEND}, {LCD_NATIVE_SPI_HZ/1e6:g}MHz): "
              f"{n if ok else 'failed'} frames in {elapsed:.2f}s "
              f"({(n/elapsed) if ok else 0:.2f} fps); verify actual LCD colors")
    else:
        ok = _run_lcd_diagnostic(long_test=True)
        print("LCD-only diagnostic:", "WRITE CALLS COMPLETED" if ok else "FAILED")
    _stop_native_lcd_bridge()
    if HAS_GPIO:
        try:
            GPIO.cleanup()
        except Exception:
            pass
    raise SystemExit(0 if ok else 1)

def _joystick_diagnostic_loop():
    """Standalone raw joystick test; no camera, YOLO, Flask, or LCD required."""
    if not HAS_JOYSTICK:
        print("Joystick diagnostic unavailable: ADS1115/joystick did not initialize.")
        return 1

    print("")
    print("=" * 66)
    print(" JOYSTICK-ONLY DIAGNOSTIC")
    print("=" * 66)
    print("Expected with joystick powered from 3.3V:")
    print("  centered: X and Y roughly ~1.65V")
    print("  full travel: each axis should move substantially toward ~0V / ~3.3V")
    print("  click: GPIO22 should change from 1 to 0")
    print("Press Ctrl+C to stop.")
    print("=" * 66)

    min_x = min_y = 99.0
    max_x = max_y = -99.0
    try:
        while True:
            xv = float(_joy_x.voltage)
            yv = float(_joy_y.voltage)
            min_x, max_x = min(min_x, xv), max(max_x, xv)
            min_y, max_y = min(min_y, yv), max(max_y, yv)
            sw = GPIO.input(JOYSTICK_SW_PIN) if HAS_GPIO else -1

            rail = ""
            if xv < 0.12 or xv > 3.18 or yv < 0.12 or yv > 3.18:
                rail = "  <RAIL>"

            print(
                f"\rX={xv:5.3f}V [{min_x:5.3f}..{max_x:5.3f}]  "
                f"Y={yv:5.3f}V [{min_y:5.3f}..{max_y:5.3f}]  "
                f"SW={sw}{rail}      ",
                end="", flush=True
            )
            time.sleep(0.12)
    except KeyboardInterrupt:
        print("\nJoystick diagnostic stopped.")
    return 0

if JOYSTICK_ONLY_TEST:
    rc = _joystick_diagnostic_loop()
    if HAS_GPIO:
        try:
            GPIO.cleanup()
        except Exception:
            pass
    raise SystemExit(rc)

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": STILL_SIZE, "format": "RGB888"},
    # Current Raspberry Pi Picamera2/libcamera builds require the lores stream
    # to be YUV. Convert it once after capture for OpenCV/PIL processing.
    lores={"size": PREVIEW_SIZE, "format": "YUV420"},
    display=None,
    encode=None,
    buffer_count=4,
    controls={"FrameDurationLimits": (16666, PREVIEW_FRAME_MAX_US)},
)
picam2.configure(config)
# Picamera2 may align a requested size.  Use the actual configured dimensions
# for all later coordinate transforms.
_actual_camera_config = picam2.camera_configuration()
STILL_SIZE = tuple(_actual_camera_config["main"]["size"])
PREVIEW_SIZE = tuple(_actual_camera_config["lores"]["size"])
_camera_capture_lock = threading.Lock()


def _capture_stream_array(stream_name):
    """Capture one stream and normalize it to an RGB H×W×3 array.

    Picamera2 requires the low-resolution stream to be YUV420 on current
    Raspberry Pi OS builds. capture_array() returns that stream as a planar
    H*3/2 × W array, so convert it here while leaving the RGB main stream
    unchanged. Keeping conversion in one place prevents every downstream
    effect, detector, and scene mapper from needing YUV-specific logic.
    """
    with _camera_capture_lock:
        frame = picam2.capture_array(stream_name)

    if stream_name == 'lores':
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2RGB_I420)
        elif frame.ndim == 3 and frame.shape[2] == 1:
            frame = cv2.cvtColor(frame[:, :, 0], cv2.COLOR_YUV2RGB_I420)
        elif frame.ndim != 3 or frame.shape[2] < 3:
            raise RuntimeError(f'unexpected lores YUV array shape: {frame.shape}')

    return np.ascontiguousarray(frame[:, :, :3])

picam2.start()
time.sleep(1)

# ─── YOLO model ───────────────────────────────────────────────────────────────
_model = None
_model_lock = threading.Lock()

def get_model():
    global _model
    if _model is None:
        with _model_lock:
            if _model is None:
                print("Loading YOLO model...")
                from ultralytics import YOLO
                _model = YOLO('yolov8n.pt')
                print("✓ YOLO model loaded")
    return _model

# ─── State ────────────────────────────────────────────────────────────────────
# Detection data is read by the frame renderer, Flask routes, and the YOLO thread.
# Keep it behind a lock so mode changes and bounding boxes do not flicker or tear.
DETECT_CONF = float(os.environ.get("DETECT_CONF", "0.22"))
DETECTION_STALE_SEC = float(os.environ.get("DETECTION_STALE_SEC", "0.35"))

# Lightweight persistent tracker.  YOLO supplies detections; this layer assigns
# stable IDs, smooths box jitter, predicts through brief missed detections, and
# prevents Erased mode from blinking when the detector drops a frame.
TRACK_IOU_MIN = float(os.environ.get("TRACK_IOU_MIN", "0.16"))
TRACK_CENTER_MAX = float(os.environ.get("TRACK_CENTER_MAX", "0.42"))
TRACK_DETECTION_WEIGHT = float(os.environ.get("TRACK_DETECTION_WEIGHT", "0.72"))
TRACK_TTL_SEC = float(os.environ.get("TRACK_TTL_SEC", "1.35"))
_track_lock = threading.Lock()
_tracks = {}
_next_track_id = 1

_detection_lock = threading.Lock()
detection_data = {
    'people': [],
    'tracks': [],
    'last_people': [],
    'count': 0,
    'total_seen': 0,
    'ghost_trails': [],
    'last_seen_ts': 0.0,
    'last_update_ts': 0.0,
    'status': 'waiting',
}

viz_settings = {
    'mode':'env_accurate', 'intensity':78, 'ghost_persistence':18, 'scan_speed':3,
    'image_tone':'bw', 'overlay_tone':'color'
}

LOCAL_MODE_ORDER = [
    'env_accurate','env_field','env_particles','env_abstract',
    'panopticon','ghost','dissolution','shield','erased','mapped','hyperspace','all','raw'
]
LOCAL_MODE_LABELS = {
    'env_accurate':'FIELD PLATE',
    'env_field':'FIELD WARP',
    'env_particles':'CLOUD CHAMBER',
    'env_abstract':'RESONANCE PLATE',
    'panopticon':'PANOPTICON',
    'ghost':'AFTERIMAGE',
    'dissolution':'EMULSION',
    'shield':'FARADAY VEIL',
    'erased':'ERASE',
    'mapped':'MAP ERASE',
    'hyperspace':'4D',
    'all':'COMPOSITE',
    'raw':'RAW CAMERA'
}
_local_ui_focus = 0
LOCAL_UI_ACTIONS = ['prev','next','intensity_down','intensity_up','image_tone','overlay_tone','capture']
_local_control_lock = threading.Lock()
tap_regions    = []
tap_regions_lock = threading.Lock()
_frame_count = 0
CAPTURE_DIR = os.environ.get("CAPTURE_DIR", "/home/stoopchild11/captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)

# Erasure settings.  Adaptive mode learns a clean plate whenever the scene is
# empty.  Mapped mode uses an explicit multi-frame scan saved to disk, which is
# the most reliable way to reveal the real pixels behind a person when the camera
# remains fixed.
ERASE_PAD = int(os.environ.get("ERASE_PAD", "10"))
ERASE_BG_LR = float(os.environ.get("ERASE_BG_LR", "0.025"))
ERASE_EMPTY_LR = float(os.environ.get("ERASE_EMPTY_LR", "0.12"))
ERASE_FEATHER = float(os.environ.get("ERASE_FEATHER", "1.8"))
ERASE_DIFF_MIN = int(os.environ.get("ERASE_DIFF_MIN", "12"))
ERASE_BG_READY_EMPTY_FRAMES = int(os.environ.get("ERASE_BG_READY_EMPTY_FRAMES", "12"))
ERASE_MASK_HISTORY = int(os.environ.get("ERASE_MASK_HISTORY", "2"))
SCENE_SCAN_TARGET = int(os.environ.get("SCENE_SCAN_TARGET", "20"))
SCENE_SCAN_MOTION_LIMIT = float(os.environ.get("SCENE_SCAN_MOTION_LIMIT", "18.0"))

# Lightweight ORB/RANSAC registration compensates for small handheld shifts,
# rotation, and scale changes before comparing/replacing pixels.
ERASE_ALIGN_WIDTH = int(np.clip(int(os.environ.get("ERASE_ALIGN_WIDTH", "360")), 240, 640))
ERASE_ALIGN_EVERY = int(np.clip(int(os.environ.get("ERASE_ALIGN_EVERY", "6")), 1, 20))
ERASE_ALIGN_FEATURES = int(np.clip(int(os.environ.get("ERASE_ALIGN_FEATURES", "320")), 120, 800))
ERASE_ALIGN_MIN_INLIERS = int(np.clip(int(os.environ.get("ERASE_ALIGN_MIN_INLIERS", "9")), 5, 40))
ERASE_ALIGN_MAX_ROT_DEG = float(np.clip(float(os.environ.get("ERASE_ALIGN_MAX_ROT_DEG", "5.0")), 1.0, 12.0))
ERASE_ALIGN_MAX_SHIFT_RATIO = float(np.clip(float(os.environ.get("ERASE_ALIGN_MAX_SHIFT_RATIO", "0.14")), 0.03, 0.30))
ERASE_ALIGN_MIN_SCALE = float(np.clip(float(os.environ.get("ERASE_ALIGN_MIN_SCALE", "0.93")), 0.80, 0.99))
ERASE_ALIGN_MAX_SCALE = float(np.clip(float(os.environ.get("ERASE_ALIGN_MAX_SCALE", "1.07")), 1.01, 1.25))
MAPPED_BACKGROUND_PATH = os.environ.get(
    "MAPPED_BACKGROUND_PATH",
    os.path.join(CAPTURE_DIR, "erasure_scene_map.png"),
)

_background_lock = threading.Lock()
_background_model = None
_background_frames = 0
_background_empty_frames = 0
_background_last_update = 0.0

_scene_map_lock = threading.Lock()
_mapped_background = None
_mapped_created_at = None
_scene_scan_sum = None
_scene_scan_prev = None
_scene_scan_reference = None
_scene_scan_finalize_thread = None
_scene_scan_state = {
    'active': False,
    'phase': 'idle',
    'status': 'not mapped',
    'accepted': 0,
    'rejected': 0,
    'target': SCENE_SCAN_TARGET,
    'percent': 0,
    'alignment': 'idle',
    'last_motion': None,
    'started_at': None,
}

_alignment_lock = threading.Lock()
_alignment_cache = {}
_alignment_status = {
    'ok': False,
    'source': None,
    'status': 'idle',
    'matches': 0,
    'inliers': 0,
    'rotation_deg': 0.0,
    'scale': 1.0,
    'shift_px': 0.0,
}

_erasure_mask_lock = threading.Lock()
_erasure_mask_history = _collections.deque(maxlen=max(1, ERASE_MASK_HISTORY))
_erasure_mask_shape = None

_capture_lock = threading.Lock()
capture_state = {
    'id': 0,
    'success': False,
    'filename': None,
    'display_name': None,
    'timestamp': None,
    'source': None,
    'error': None,
}

# Full-screen photo review is always handled by the LCD worker (sole SPI writer).
_photo_lock = threading.Lock()
_photo_status = 'live'  # live, saving, review, error
_photo_file = None
_photo_image = None
_photo_message = ''
_photo_until = 0.0
_photo_revision = 0
_photo_save_lock = threading.Lock()  # ignore bounce/repeated presses during a save


def _set_photo_state(status, filename=None, message='', image=None):
    global _photo_status, _photo_file, _photo_image, _photo_message, _photo_until, _photo_revision
    with _photo_lock:
        _photo_status = status
        _photo_file = filename
        _photo_image = image
        _photo_message = message
        _photo_until = (time.monotonic() + PHOTO_REVIEW_SEC if status in ('review', 'error') and PHOTO_REVIEW_SEC > 0 else 0)
        _photo_revision += 1


def _close_photo_review():
    _set_photo_state('live')
    _touch_local_ui()


def _review_photo_step(step):
    with _photo_lock:
        current = _photo_file
        active = _photo_status == 'review'
    if not active:
        return
    files = sorted(Path(CAPTURE_DIR).glob('*.jpg'))
    if not files:
        return
    try:
        index = files.index(Path(current))
    except (ValueError, TypeError):
        index = len(files) - 1
    target = files[(index + int(step)) % len(files)]
    try:
        with Image.open(target) as im:
            thumb = ImageOps.contain(im.convert('RGB'), (_lcd_width, _lcd_height - 30), Image.Resampling.BILINEAR).copy()
        _set_photo_state('review', str(target), 'SAVED PHOTO', thumb)
    except (OSError, ValueError) as exc:
        _set_photo_state('error', message=f'PHOTO READ ERROR: {exc}')


def _review_canvas():
    with _photo_lock:
        status, image, filename, message, until = _photo_status, _photo_image, _photo_file, _photo_message, _photo_until
    if status == 'live':
        return None
    if status in ('review', 'error') and until and time.monotonic() >= until:
        _close_photo_review()
        return None
    w, h = _lcd_width, _lcd_height
    canvas = Image.new('RGB', (w, h), (5, 8, 11))
    draw = ImageDraw.Draw(canvas)
    if status == 'review' and image is not None:
        canvas.paste(image, ((w-image.width)//2, max(0,(h-30-image.height)//2)))
        draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, h-30, w, h), fill=(8, 26, 20) if status == 'review' else (44, 20, 16))
    title = 'SAVED  ✓' if status == 'review' else ('SAVING PHOTO...' if status == 'saving' else 'CAPTURE ERROR')
    draw.text((7, h-27), title, fill=(150,255,195) if status == 'review' else (255,230,175))
    if status == 'review':
        draw.text((115, h-26), '← → photos  CLICK live', fill=(225,225,230))
        if filename:
            draw.text((7, h-13), Path(filename).name[:47], fill=(170,180,185))
    elif message:
        draw.text((7, h-13), str(message)[:49], fill=(255,195,190))
    return canvas


def _hardware_photo_capture():
    # A single capture at a time. The UI indicates saving immediately.
    if not _photo_save_lock.acquire(blocking=False):
        return
    _set_photo_state('saving', message='Processing high-resolution still')
    try:
        result = capture_current_frame(prefix='resonance_shutter', source='hardware')
        if result.get('success'):
            filename = result['filename']
            try:
                with Image.open(filename) as im:
                    thumb = ImageOps.contain(im.convert('RGB'), (_lcd_width, _lcd_height-30), Image.Resampling.BILINEAR).copy()
            except Exception as exc:
                print('Photo thumbnail unavailable:', exc)
                thumb = None
            _set_photo_state('review', filename, image=thumb)
            print(f'✓ Physical shutter captured and saved: {filename}', flush=True)
        else:
            error = result.get('error','Unknown error')
            _set_photo_state('error', message=str(error))
            print(f'✗ Physical shutter capture failed: {error}', flush=True)
    finally:
        _photo_save_lock.release()


def _notify_capture(success, filename=None, source='web', error=None):
    """Expose hardware/web captures to the mobile UI polling loop."""
    with _capture_lock:
        capture_state['id'] += 1
        capture_state['success'] = bool(success)
        capture_state['filename'] = filename
        capture_state['display_name'] = os.path.basename(filename) if filename else None
        capture_state['timestamp'] = datetime.now().isoformat(timespec='seconds')
        capture_state['source'] = source
        capture_state['error'] = error


# ─── Detection thread ─────────────────────────────────────────────────────────
_latest_frame = None
_latest_frame_id = 0
_frame_lock   = threading.Lock()

def _clip_box(x1, y1, x2, y2, w, h):
    x1 = max(0, min(int(x1), w - 1))
    y1 = max(0, min(int(y1), h - 1))
    x2 = max(0, min(int(x2), w - 1))
    y2 = max(0, min(int(y2), h - 1))
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)


def _box_iou(a, b):
    ax1, ay1, ax2, ay2 = map(float, a)
    bx1, by1, bx2, by2 = map(float, b)
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(1.0, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1.0, (bx2 - bx1) * (by2 - by1))
    return inter / max(1.0, area_a + area_b - inter)


def _center_distance_ratio(a, b, frame_diag):
    acx, acy = (a[0] + a[2]) * 0.5, (a[1] + a[3]) * 0.5
    bcx, bcy = (b[0] + b[2]) * 0.5, (b[1] + b[3]) * 0.5
    return math.hypot(acx - bcx, acy - bcy) / max(1.0, frame_diag)


def _clip_box_array(box, w, h):
    clipped = _clip_box(box[0], box[1], box[2], box[3], w, h)
    if clipped is None:
        return None
    return np.array(clipped, dtype=np.float32)


def _update_person_tracks(detections, now, frame_w, frame_h):
    """Associate YOLO detections with persistent, smoothed person tracks."""
    global _next_track_id
    frame_diag = math.hypot(frame_w, frame_h)
    detections = [
        {'box': np.array(d[:4], dtype=np.float32), 'conf': float(d[4])}
        for d in detections
    ]

    with _track_lock:
        # Predict where each live track should be for this detector update.
        for tr in _tracks.values():
            tr['predicted_box'] = tr['box'] + tr['velocity']

        candidates = []
        for tid, tr in _tracks.items():
            pred = tr['predicted_box']
            for di, det in enumerate(detections):
                iou = _box_iou(pred, det['box'])
                dist = _center_distance_ratio(pred, det['box'], frame_diag)
                # Prefer overlap, but allow a low-overlap match when a person
                # moved quickly and the centers are still plausibly close.
                if iou >= TRACK_IOU_MIN or dist <= TRACK_CENTER_MAX * 0.25:
                    score = iou - (dist * 0.35)
                    candidates.append((score, tid, di))

        candidates.sort(reverse=True)
        matched_tracks = set()
        matched_dets = set()
        for _score, tid, di in candidates:
            if tid in matched_tracks or di in matched_dets:
                continue
            tr = _tracks.get(tid)
            if tr is None:
                continue
            det = detections[di]
            old_box = tr['box'].copy()
            predicted = tr['predicted_box']
            smoothed = predicted * (1.0 - TRACK_DETECTION_WEIGHT) + det['box'] * TRACK_DETECTION_WEIGHT
            clipped = _clip_box_array(smoothed, frame_w, frame_h)
            if clipped is None:
                continue
            measured_velocity = clipped - old_box
            tr['velocity'] = tr['velocity'] * 0.58 + measured_velocity * 0.42
            tr['box'] = clipped
            tr['confidence'] = det['conf']
            tr['last_seen'] = now
            tr['predicted'] = False
            tr['hits'] += 1
            matched_tracks.add(tid)
            matched_dets.add(di)

        # Carry unmatched tracks briefly using damped velocity.
        expired = []
        for tid, tr in _tracks.items():
            if tid in matched_tracks:
                continue
            age = now - tr['last_seen']
            if age > TRACK_TTL_SEC:
                expired.append(tid)
                continue
            predicted = _clip_box_array(tr['predicted_box'], frame_w, frame_h)
            if predicted is None:
                expired.append(tid)
                continue
            tr['box'] = predicted
            tr['velocity'] *= 0.72
            tr['confidence'] *= 0.90
            tr['predicted'] = True

        for tid in expired:
            _tracks.pop(tid, None)

        new_tracks = 0
        for di, det in enumerate(detections):
            if di in matched_dets:
                continue
            clipped = _clip_box_array(det['box'], frame_w, frame_h)
            if clipped is None:
                continue
            tid = _next_track_id
            _next_track_id += 1
            _tracks[tid] = {
                'id': tid,
                'box': clipped,
                'predicted_box': clipped.copy(),
                'velocity': np.zeros(4, dtype=np.float32),
                'confidence': det['conf'],
                'last_seen': now,
                'predicted': False,
                'hits': 1,
            }
            new_tracks += 1

        visible = []
        details = []
        for tid, tr in _tracks.items():
            box = tr['box']
            conf = max(0.08, float(tr['confidence']))
            visible.append((int(box[0]), int(box[1]), int(box[2]), int(box[3]), conf))
            details.append({
                'id': int(tid),
                'box': [int(box[0]), int(box[1]), int(box[2]), int(box[3])],
                'confidence': round(conf, 4),
                'predicted': bool(tr['predicted']),
                'hits': int(tr['hits']),
            })

    visible.sort(key=lambda b: ((b[2]-b[0])*(b[3]-b[1]), b[4]), reverse=True)
    details.sort(key=lambda t: t['id'])
    return visible, details, new_tracks


def _smooth_people_for_render():
    """Return current detections, briefly holding the last good boxes.

    YOLO can miss a frame or two on the Pi.  Holding boxes for a fraction of a
    second makes Erased, Ghost, Shield, and Hyperspace feel continuous instead
    of blinking off between detections.
    """
    now = time.monotonic()
    with _detection_lock:
        people = list(detection_data.get('people', []))
        trails = [list(t) for t in detection_data.get('ghost_trails', [])]
        last_people = list(detection_data.get('last_people', []))
        last_seen_ts = float(detection_data.get('last_seen_ts', 0.0))
        total_seen = int(detection_data.get('total_seen', 0))
        status = detection_data.get('status', 'waiting')

    stale = False
    if not people and last_people and (now - last_seen_ts) <= DETECTION_STALE_SEC:
        # Fade confidence while held, but keep the boxes stable for rendering.
        age = max(0.0, now - last_seen_ts)
        fade = max(0.25, 1.0 - age / max(DETECTION_STALE_SEC, 0.001))
        people = [(x1, y1, x2, y2, float(conf) * fade) for (x1, y1, x2, y2, conf) in last_people]
        stale = True
    return people, trails, total_seen, status, stale


def detection_loop():
    global detection_data
    model = get_model()
    last_frame_id = -1
    period = 1.0 / max(1.0, DETECT_FPS)
    next_run = time.monotonic()
    while not _shutdown_event.is_set():
        now_wait = time.monotonic()
        if now_wait < next_run:
            time.sleep(min(0.05, next_run - now_wait))
            continue

        frame = None
        frame_id = last_frame_id
        with _frame_lock:
            if _latest_frame is not None and _latest_frame_id != last_frame_id:
                frame = _latest_frame.copy()
                frame_id = _latest_frame_id
        if frame is None:
            time.sleep(0.03)
            continue

        last_frame_id = frame_id
        next_run = time.monotonic() + period
        try:
            fh, fw = frame.shape[:2]
            results = model(frame[:, :, :3], classes=[0], conf=DETECT_CONF, imgsz=320, verbose=False)
            detections = []
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    clipped = _clip_box(x1, y1, x2, y2, fw, fh)
                    if clipped is None:
                        continue
                    detections.append((*clipped, float(box.conf[0])))

            now = time.monotonic()
            people, tracks, new_track_count = _update_person_tracks(detections, now, fw, fh)
            actual_count = len(detections)
            predicted_count = sum(1 for t in tracks if t.get('predicted'))

            with _detection_lock:
                detection_data['people'] = people
                detection_data['tracks'] = tracks
                detection_data['count'] = len(people)
                detection_data['last_update_ts'] = now
                if actual_count:
                    detection_data['status'] = f'tracking {actual_count}'
                elif people:
                    detection_data['status'] = f'predicting {predicted_count}'
                else:
                    detection_data['status'] = 'searching'
                if people:
                    detection_data['last_people'] = list(people)
                    detection_data['last_seen_ts'] = now
                detection_data['total_seen'] += new_track_count
                max_trail = int(np.clip(viz_settings.get('ghost_persistence', 12), 2, 30))
                detection_data['ghost_trails'].append(list(people))
                if len(detection_data['ghost_trails']) > max_trail:
                    detection_data['ghost_trails'] = detection_data['ghost_trails'][-max_trail:]
        except Exception as e:
            with _detection_lock:
                detection_data['status'] = f'error: {e}'
            print(f"Detection error: {e}")

if ENABLE_DETECTION:
    threading.Thread(target=detection_loop, daemon=True, name="person-detector").start()
else:
    detection_data['status'] = 'disabled' 

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  TAP-TO-REMOVE
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def apply_tap_removals(img):
    with tap_regions_lock:
        regions = list(tap_regions)
    if not regions:
        return img
    arr = np.array(img.convert('RGB'))
    h, w = arr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    for (nx1, ny1, nx2, ny2) in regions:
        x1, y1 = int(nx1*w), int(ny1*h)
        x2, y2 = int(nx2*w), int(ny2*h)
        pad = 6
        cv2.rectangle(mask, (max(0,x1-pad), max(0,y1-pad)), (min(w-1,x2+pad), min(h-1,y2+pad)), 255, -1)

    # In erasure modes, manual regions also use the real clean plate.  Other
    # visual modes retain local inpainting because their processed pixels no
    # longer match the clean camera background.
    mode = viz_settings.get('mode', 'panopticon')
    source = 'mapped' if mode == 'mapped' else 'adaptive'
    bg, bg_ready, _label = get_erasure_background((w, h), source=source)
    if mode in ('erased', 'mapped') and bg_ready and bg is not None:
        result = _replace_with_clean_plate(arr, bg, mask)
    else:
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        result = cv2.cvtColor(cv2.inpaint(bgr, mask, 7, cv2.INPAINT_TELEA), cv2.COLOR_BGR2RGB)

    out  = Image.fromarray(result).convert('RGBA')
    over = Image.new('RGBA', (w, h), (0,0,0,0))
    draw = ImageDraw.Draw(over)
    for (nx1, ny1, nx2, ny2) in regions:
        x1, y1 = int(nx1*w), int(ny1*h)
        x2, y2 = int(nx2*w), int(ny2*h)
        draw.rectangle([(x1,y1),(x2,y2)], outline=(0,255,80,200), width=2)
    return Image.alpha_composite(out, over).convert('RGB')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  VISUAL EFFECTS (original modes)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_font_cache = {}
_font_cache_lock = threading.Lock()

def _font(size=10):
    size = int(size)
    with _font_cache_lock:
        cached = _font_cache.get(size)
        if cached is not None:
            return cached
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", size)
        except Exception:
            font = ImageFont.load_default()
        _font_cache[size] = font
        return font


def _clamp_int(v, lo=0, hi=255):
    return int(max(lo, min(hi, v)))


def _safe_people(people, w, h):
    safe = []
    for p in people:
        if len(p) < 5:
            continue
        x1, y1, x2, y2, conf = p
        x1 = _clamp_int(x1, 0, max(0, w - 1))
        y1 = _clamp_int(y1, 0, max(0, h - 1))
        x2 = _clamp_int(x2, 0, max(0, w - 1))
        y2 = _clamp_int(y2, 0, max(0, h - 1))
        if x2 > x1 + 2 and y2 > y1 + 2:
            safe.append((x1, y1, x2, y2, float(conf)))
    return safe


def _glow_text(draw, xy, text, font, fill=(0,255,100,220), glow=(0,0,0,210), radius=1):
    x, y = xy
    for ox in range(-radius, radius + 1):
        for oy in range(-radius, radius + 1):
            if ox or oy:
                draw.text((x + ox, y + oy), text, font=font, fill=glow)
    draw.text((x, y), text, font=font, fill=fill)


def _mode_stamp(layer, title, subtitle, frame_num, s, accent=(0,255,120)):
    draw = ImageDraw.Draw(layer)
    w, h = layer.size
    font_title = _font(12)
    font_body = _font(9)
    alpha = int(170 * s)
    draw.rounded_rectangle([(10, h - 48), (min(w - 10, 330), h - 10)], radius=7, fill=(4, 8, 10, int(132 * s)), outline=(*accent, int(65 * s)), width=1)
    _glow_text(draw, (18, h - 42), title, font_title, fill=(*accent, alpha), radius=1)
    draw.text((18, h - 25), subtitle, font=font_body, fill=(180, 210, 205, int(130 * s)))
    tick_x = 305 if w > 340 else w - 34
    draw.line([(tick_x, h - 37), (tick_x + int(15 * math.sin(frame_num * 0.18)), h - 37)], fill=(*accent, int(150 * s)), width=2)


def _film_grain(arr, amount=5):
    if amount <= 0:
        return arr
    noise = np.random.normal(0, amount, arr.shape).astype(np.int16)
    return np.clip(arr.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def _vignette_rgb(arr, strength=0.22):
    h, w = arr.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    xx = (xx / max(1, w - 1)) * 2.0 - 1.0
    yy = (yy / max(1, h - 1)) * 2.0 - 1.0
    r2 = xx * xx + yy * yy
    v = np.clip(1.0 - strength * r2, 0.45, 1.0)
    return np.clip(arr.astype(np.float32) * v[:, :, None], 0, 255).astype(np.uint8)


def _chromatic_aberration(arr, frame_num, amount=3):
    if amount <= 0:
        return arr
    out = arr.copy()
    shift_x = int(round(amount * math.sin(frame_num * 0.11))) or 1
    shift_y = int(round(amount * math.cos(frame_num * 0.07))) or 1
    out[:, :, 0] = np.roll(arr[:, :, 0], shift_x, axis=1)
    out[:, :, 2] = np.roll(arr[:, :, 2], -shift_y, axis=0)
    return out


def _science_plate_base(img, strength=0.55):
    """Scientific-photographic base: restrained local contrast, grain, vignette."""
    arr = np.array(img.convert("RGB"), dtype=np.uint8)
    gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
    clahe = cv2.createCLAHE(clipLimit=1.35, tileGridSize=(6, 6))
    plate = clahe.apply(gray)
    rgb = cv2.cvtColor(plate, cv2.COLOR_GRAY2RGB).astype(np.float32)
    src = arr.astype(np.float32)
    mix = float(np.clip(strength, 0.0, 1.0))
    out = src * (1.0 - mix) + rgb * mix
    noise = np.random.normal(0.0, 1.6 + 2.0 * mix, out.shape[:2]).astype(np.float32)
    out += noise[:, :, None]
    return _vignette_rgb(np.clip(out, 0, 255).astype(np.uint8), 0.12 + 0.12 * mix)


def _fine_contours(layer, source_img, alpha=72):
    """Sparse image-derived contour drawing, closer to a scientific plate than a HUD."""
    arr = np.array(source_img.convert("L"), dtype=np.uint8)
    arr = cv2.GaussianBlur(arr, (0, 0), 1.2)
    edges = cv2.Canny(arr, 55, 120)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    draw = ImageDraw.Draw(layer)
    for c in contours[:120]:
        if len(c) < 8:
            continue
        stride = max(1, len(c) // 44)
        pts = [(int(p[0][0]), int(p[0][1])) for p in c[::stride]]
        if len(pts) >= 2:
            draw.line(pts, fill=(226, 233, 226, int(alpha)), width=1)


def _field_streamlines(layer, theta, drive, frame_num, intensity):
    """Sparse magnetic streamlines rather than a generic vector grid."""
    draw = ImageDraw.Draw(layer)
    w, h = layer.size
    s = float(np.clip(float(intensity) / 100.0, 0.0, 1.0))
    drive = float(np.clip(drive, 0.0, 1.0))
    rng = random.Random(4811 + frame_num // 3)
    count = int(8 + 9 * s)
    phase = frame_num * (0.012 + 0.024 * drive)
    for idx in range(count):
        y0 = (idx + 0.5) * h / count + rng.uniform(-4, 4)
        pts = []
        for j in range(46):
            u = j / 45.0
            x = -0.08 * w + u * 1.16 * w
            y = y0 + math.sin(u * math.pi * (2.0 + 2.4 * drive) + phase + idx * 0.31) * (5 + 21 * drive)
            cx, cy = w * 0.5, h * 0.5
            ang = theta * 0.30
            xr = (x - cx) * math.cos(ang) - (y - cy) * math.sin(ang) + cx
            yr = (x - cx) * math.sin(ang) + (y - cy) * math.cos(ang) + cy
            pts.append((xr, yr))
        a = int((52 + 80 * (idx % 3 == 0)) * (0.35 + 0.65 * s))
        draw.line(pts, fill=(214, 234, 229, a), width=1)


def _cloud_track(rng, w, h, kind, length, curvature, handedness):
    """Schematic cloud-chamber track geometry."""
    if kind == "cosmic":
        side = rng.choice(("l", "r", "t", "b"))
        if side == "l":
            x, y, ang = -10.0, rng.uniform(0, h), rng.uniform(-0.2, 0.2)
        elif side == "r":
            x, y, ang = w + 10.0, rng.uniform(0, h), math.pi + rng.uniform(-0.2, 0.2)
        elif side == "t":
            x, y, ang = rng.uniform(0, w), -10.0, math.pi/2 + rng.uniform(-0.2, 0.2)
        else:
            x, y, ang = rng.uniform(0, w), h + 10.0, -math.pi/2 + rng.uniform(-0.2, 0.2)
        steps = max(22, int(length / 5))
        step = max(4.5, length / steps)
    else:
        x, y = rng.uniform(0.06*w, 0.94*w), rng.uniform(0.08*h, 0.92*h)
        ang = rng.uniform(0, math.tau)
        steps = max(12, int(length / (4.5 if kind == "alpha" else 3.5)))
        step = max(2.2, length / steps)

    pts = []
    for _ in range(steps):
        pts.append((x, y))
        if kind == "beta":
            ang += rng.gauss(0, 0.075) + curvature * handedness * 0.018
        elif kind == "alpha":
            ang += rng.gauss(0, 0.014) + curvature * handedness * 0.004
        else:
            ang += curvature * handedness * 0.002
        ds = step * rng.uniform(0.84, 1.16)
        x += math.cos(ang) * ds
        y += math.sin(ang) * ds
        if x < -15 or x > w + 15 or y < -15 or y > h + 15:
            break
    return pts


def _draw_cloud_chamber(layer, snap, frame_num, intensity, dense=False):
    """Cloud-chamber-inspired traces driven by live environmental measurements.

    The track shapes are artistic/scientific encodings; the camera is not being
    presented as directly identifying particle species.
    """
    draw = ImageDraw.Draw(layer)
    w, h = layer.size
    sw = snap.get("swpc", {})
    cosmic = snap.get("cosmic_ray", {})
    md, pd = _env_drives(snap)
    s = float(np.clip(float(intensity) / 100.0, 0.0, 1.0))

    speed = _float_or_none(sw.get("solar_wind_speed")) or 350.0
    bz = _float_or_none(sw.get("bz_gsm")) or 0.0
    kp = _float_or_none(sw.get("kp")) or 0.0
    proton = max(0.0, _float_or_none(sw.get("proton_flux_10mev")) or 0.0)
    cosmic_rel = _float_or_none(cosmic.get("relative_percent")) or 0.0

    density = float(np.clip(pd + abs(cosmic_rel)/8.0, 0.0, 1.6))
    persistence = 8 if not dense else 11
    tracks = int((5 if not dense else 8) + (16 if not dense else 26) * density * s)
    length_scale = float(np.clip((speed - 250.0)/450.0, 0.18, 1.35))
    curvature = float(np.clip(abs(bz)/18.0 + kp/14.0, 0.05, 1.1))
    handedness = -1.0 if bz < 0 else 1.0
    proton_weight = float(np.clip(math.log10(1.0 + proton)/3.0, 0.08, 0.58))

    # Low-frequency chamber vapor substrate.
    rng_np = np.random.default_rng(7300 + frame_num//2)
    fh, fw = max(8, h//18), max(8, w//18)
    fog = rng_np.normal(0, 1, (fh, fw)).astype(np.float32)
    fog = cv2.resize(fog, (w, h), interpolation=cv2.INTER_CUBIC)
    fog = cv2.GaussianBlur(fog, (0,0), 5)
    fog = (fog - fog.min()) / max(1e-6, fog.max()-fog.min())
    alpha = np.clip((fog - 0.48) * (24 + 30*s), 0, 30).astype(np.uint8)
    fog_img = Image.new("RGBA", (w, h), (205, 218, 213, 0))
    fog_img.putalpha(Image.fromarray(alpha, mode="L"))
    layer.alpha_composite(fog_img)

    per_age = max(1, tracks // max(1, persistence//2))
    for age in range(persistence-1, -1, -1):
        rng = random.Random(91021 + (frame_num//2 - age)*101)
        fade = (1.0 - age/max(1,persistence))**1.35
        a = int((38 + 175*fade) * (0.4 + 0.6*s))
        for _ in range(per_age):
            r = rng.random()
            if r < 0.10 + 0.08*abs(cosmic_rel)/5.0:
                kind = "cosmic"
                length = max(w,h)*rng.uniform(0.9,1.5)
                width = 1
            elif r < 0.10 + proton_weight:
                kind = "alpha"
                length = min(w,h)*rng.uniform(0.16,0.34)*(0.82+0.32*length_scale)
                width = 2 if s > 0.5 else 1
            else:
                kind = "beta"
                length = min(w,h)*rng.uniform(0.24,0.66)*(0.78+0.48*length_scale)
                width = 1

            pts = _cloud_track(rng, w, h, kind, length, curvature, handedness)
            if len(pts) < 2:
                continue
            col = (
                (243, 229, 191, a) if kind == "alpha"
                else (218, 239, 241, min(235,a+24)) if kind == "cosmic"
                else (227, 237, 229, a)
            )
            draw.line(pts, fill=col, width=width)
            stride = 2 if kind == "alpha" else (4 if kind == "beta" else 7)
            for px, py in pts[::stride]:
                rr = 1
                draw.ellipse((px-rr,py-rr,px+rr,py+rr),
                             fill=(236,242,232,max(18,a//2)))

            # Occasional interpretive secondary track.
            if kind != "cosmic" and len(pts) > 10 and rng.random() < 0.018 + 0.05*np.clip(kp/9.0,0,1):
                bx, by = pts[len(pts)//2]
                baseang = rng.uniform(0, math.tau)
                for sign in (-1,1):
                    bl = rng.uniform(12,30)*(0.65+s)
                    ex = bx + math.cos(baseang + sign*0.55)*bl
                    ey = by + math.sin(baseang + sign*0.55)*bl
                    draw.line((bx,by,ex,ey), fill=(222,234,225,max(22,a//2)), width=1)

    # Instrument fiducials
    tick = (210,220,214,int(72*s))
    for x in range(10,w,20):
        draw.line((x,h-7,x,h-3), fill=tick, width=1)
    for y in range(10,h,20):
        draw.line((3,y,7,y), fill=tick, width=1)


def _small_science_annotation(layer, snap, title, intensity):
    draw = ImageDraw.Draw(layer)
    w, h = layer.size
    sw = snap.get("swpc", {})
    gm = snap.get("geomag", {})
    cosmic = snap.get("cosmic_ray", {})
    vals = []
    kp = _float_or_none(sw.get("kp"))
    bz = _float_or_none(sw.get("bz_gsm"))
    df = _float_or_none(gm.get("df"))
    cr = _float_or_none(cosmic.get("relative_percent"))
    if kp is not None: vals.append(f"Kp {kp:.1f}")
    if bz is not None: vals.append(f"Bz {bz:+.1f}nT")
    if df is not None: vals.append(f"dF {df:+.0f}nT")
    if cr is not None: vals.append(f"{cosmic.get('station') or 'NM'} {cr:+.2f}%")
    font = _font(max(8, min(10, w//58)))
    draw.rectangle((8,8,min(w-8,240),27), fill=(3,6,7,105))
    draw.text((13,12), title, font=font, fill=(224,234,229,205))
    text = "  ".join(vals[:3]) if vals else "LIVE SOURCES ACQUIRING"
    boxw = min(w-16, max(150, len(text)*6+18))
    draw.rectangle((8,h-25,8+boxw,h-8), fill=(3,6,7,96))
    draw.text((13,h-22), text, font=font, fill=(210,224,219,185))


def fx_panopticon(img, people, frame_num, intensity):
    """Institutional surveillance aesthetic: grid, metrics, subject IDs."""
    base = img.copy().convert('RGBA')
    layer = Image.new('RGBA', base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    w, h = img.size
    s = intensity / 100.0
    font = _font(9)
    font_big = _font(12)
    people = _safe_people(people, w, h)

    # Dense calibration grid with radial tick marks.
    scan_y = int((frame_num * max(1, int(viz_settings.get('scan_speed', 2))) * 3) % h)
    grid_a = int(28 * s)
    for gx in range(0, w, 32):
        a = grid_a + (10 if gx % 96 == 0 else 0)
        draw.line([(gx, 0), (gx, h)], fill=(0, 220, 110, a), width=1)
    for gy in range(0, h, 32):
        a = grid_a + (10 if gy % 96 == 0 else 0)
        draw.line([(0, gy), (w, gy)], fill=(0, 220, 110, a), width=1)
    draw.line([(0, scan_y), (w, scan_y)], fill=(0, 255, 115, int(128 * s)), width=2)
    for off in range(1, 20):
        yy = scan_y - off * 3
        if 0 <= yy < h:
            draw.line([(0, yy), (w, yy)], fill=(0, 255, 115, int(75 * s * (1 - off / 20))), width=1)

    cx, cy = w // 2, h // 2
    for r in range(60, max(w, h), 90):
        draw.ellipse([(cx-r, cy-r), (cx+r, cy+r)], outline=(0, 180, 100, int(22*s)), width=1)
    draw.line([(cx - 20, cy), (cx + 20, cy)], fill=(255, 80, 45, int(120*s)), width=1)
    draw.line([(cx, cy - 20), (cx, cy + 20)], fill=(255, 80, 45, int(120*s)), width=1)

    # Subject boxes.
    for i, (x1, y1, x2, y2, conf) in enumerate(people):
        subject_id = f"SUBJ-{(frame_num * 7 + i * 113) % 9999:04d}"
        col = (255, 64, 48, int(225 * s))
        hot = (255, 190, 64, int(220 * s))
        sz = max(12, min(28, (x2 - x1) // 5))
        # Brackets.
        corners = [
            ((x1, y1), (x1 + sz, y1), (x1, y1 + sz)),
            ((x2, y1), (x2 - sz, y1), (x2, y1 + sz)),
            ((x1, y2), (x1 + sz, y2), (x1, y2 - sz)),
            ((x2, y2), (x2 - sz, y2), (x2, y2 - sz)),
        ]
        for a, b, c in corners:
            draw.line([a, b], fill=col, width=2)
            draw.line([a, c], fill=col, width=2)
        ccx, ccy = (x1 + x2) // 2, (y1 + y2) // 2
        draw.line([(ccx - 15, ccy), (ccx + 15, ccy)], fill=hot, width=1)
        draw.line([(ccx, ccy - 15), (ccx, ccy + 15)], fill=hot, width=1)
        for yy in range(y1, y2, 7):
            la = int((20 + 28 * (0.5 + 0.5 * math.sin(yy * 0.2 + frame_num * 0.35))) * s)
            draw.line([(x1, yy), (x2, yy)], fill=(255, 70, 55, la), width=1)
        _glow_text(draw, (x1, max(0, y1 - 16)), subject_id, font, fill=(255, 92, 70, int(215*s)), radius=1)
        draw.text((x1, min(h - 12, y2 + 3)), f"{conf:.0%} / BIOMETRIC LOCK", fill=(255, 220, 90, int(175*s)), font=font)

    # Header bar.
    draw.rectangle([(0, 0), (w, 26)], fill=(0, 0, 0, int(195*s)))
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    draw.text((8, 7), f"LIVE STILL  ●  {ts}  /  SUBJECTS {len(people)}  /  TOTAL {detection_data.get('total_seen',0)}", fill=(255, 72, 56, int(230*s)), font=font_big)
    _mode_stamp(layer, "PANOPTICON", "institutional gaze / tracking lattice", frame_num, s, accent=(255, 70, 48))
    return Image.alpha_composite(base, layer).convert('RGB')


def fx_ghost(img, people, ghost_trails, frame_num, intensity):
    """Person as unstable afterimage: partial removal, trails, echo fields."""
    arr = np.array(img.convert('RGB'), dtype=np.float32)
    h, w = arr.shape[:2]
    s = intensity / 100.0
    people = _safe_people(people, w, h)

    # Ambient spectral shift so the mode is alive before detection locks.
    drift = int(1 + 8 * s * (0.5 + 0.5 * math.sin(frame_num * 0.18)))
    shifted = arr.copy()
    shifted[:, :, 1] = np.roll(shifted[:, :, 1], drift, axis=1)
    shifted[:, :, 2] = np.roll(shifted[:, :, 2], -drift, axis=0)
    arr = arr * (1 - 0.16 * s) + shifted * (0.16 * s)

    bg, _, bg_empty = get_background_image((w, h))
    bg_ok = bg is not None and bg_empty >= max(3, ERASE_BG_READY_EMPTY_FRAMES // 2)

    for (x1, y1, x2, y2, conf) in people:
        region = arr[y1:y2, x1:x2]
        if region.size == 0:
            continue
        grey = np.mean(region, axis=2, keepdims=True)
        if bg_ok:
            bg_region = bg[y1:y2, x1:x2].astype(np.float32)
            # Let the background bleed through the body, but keep a trace.
            ghost = bg_region * (0.62 * s) + region * (1 - 0.62 * s)
            ghost = ghost * (1 - 0.18 * s) + np.dstack([grey, grey * 1.12, grey * 1.05]) * (0.18 * s)
            arr[y1:y2, x1:x2] = ghost
        else:
            blend = min(0.78, 0.45 + 0.35 * s)
            arr[y1:y2, x1:x2] = region * (1 - blend) + (grey * 1.25 + 35) * blend

    # Trails from prior boxes.
    for ti, trail in enumerate(ghost_trails[-30:]):
        age = len(ghost_trails[-30:]) - ti
        fade = max(0.0, 1.0 - age / max(2, len(ghost_trails[-30:]) + 1)) * s * 0.58
        if fade < 0.02:
            continue
        for (x1, y1, x2, y2, conf) in _safe_people(trail, w, h):
            dx = int(math.sin(frame_num * 0.1 + age) * 6 * s)
            dy = -int(age * 0.7)
            xx1, yy1, xx2, yy2 = _expanded_box(x1 + dx, y1 + dy, x2 + dx, y2 + dy, w, h, 0)
            if xx2 > xx1 and yy2 > yy1:
                arr[yy1:yy2, xx1:xx2] = arr[yy1:yy2, xx1:xx2] * (1 - fade) + np.array([55, 180, 130], dtype=np.float32) * fade

    out = np.clip(arr, 0, 255).astype(np.uint8)
    out = _chromatic_aberration(out, frame_num, amount=int(1 + 4*s))
    result = Image.fromarray(out).convert('RGBA')
    layer = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    font = _font(11)
    big = _font(22)
    if people:
        for (x1, y1, x2, y2, conf) in people:
            if frame_num % 10 < 7:
                _glow_text(draw, ((x1+x2)//2 - 58, (y1+y2)//2 - 12), "NOT FOUND", big, fill=(200,255,215,int(150*s)), radius=2)
            draw.rectangle([(x1, y1), (x2, y2)], outline=(120, 255, 180, int(65*s)), width=1)
    else:
        draw.text((16, h - 70), "NO SUBJECT / ECHO FIELD ACTIVE", fill=(165, 255, 205, int(95*s)), font=font)
    _mode_stamp(layer, "GHOST", "identity becomes afterimage", frame_num, s, accent=(120, 255, 180))
    return Image.alpha_composite(result, layer).convert('RGB')


def fx_dissolution(img, people, frame_num, intensity):
    """EMULSION: photographic grain/material loss rather than RGB glitch blocks."""
    arr = np.array(img.convert("RGB"),dtype=np.uint8)
    h,w = arr.shape[:2]
    ss = float(np.clip(float(intensity)/100.0,0.0,1.0))
    safe = _safe_people(people,w,h)
    gray = cv2.cvtColor(arr,cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray,(0,0),1.1),45,115)
    rng = np.random.default_rng(12000+frame_num//2)

    mask=np.zeros((h,w),dtype=np.float32)
    for x1,y1,x2,y2,_ in safe:
        yy,xx=np.mgrid[y1:y2,x1:x2]
        cx,cy=(x1+x2)/2.0,(y1+y2)/2.0
        rx,ry=max(1,(x2-x1)/2),max(1,(y2-y1)/2)
        body=np.clip(1.0-(((xx-cx)/rx)**2+((yy-cy)/ry)**2),0,1)
        mask[y1:y2,x1:x2]=np.maximum(mask[y1:y2,x1:x2],body)
    if not safe:
        mask[:]=0.16

    speck=rng.random((h,w))
    voids=speck > (1.0-(0.025+0.13*ss)*mask)
    voids |= (edges>0) & (rng.random((h,w)) < (0.035+0.12*ss)*np.maximum(mask,0.1))

    out=arr.astype(np.float32)
    paper=np.dstack([gray*.92+28,gray*.95+31,gray*.90+27]).astype(np.float32)
    blend=np.clip(mask*(0.15+0.60*ss),0,0.82)[:,:,None]
    out=out*(1-blend)+paper*blend
    out[voids]=np.clip(out[voids]*0.25+185,0,255)

    # Vertical darkroom-like streaking.
    for x in range((frame_num*3)%17,w,max(12,int(28-10*ss))):
        out[:,x:x+1]=np.clip(out[:,x:x+1]*(0.82-0.13*ss)+22,0,255)

    base=Image.fromarray(np.clip(out,0,255).astype(np.uint8)).convert("RGBA")
    layer=Image.new("RGBA",base.size,(0,0,0,0))
    draw=ImageDraw.Draw(layer)
    for x1,y1,x2,y2,_ in safe:
        pad=max(2,int(5+10*ss))
        for j in range(3):
            inset=j*max(2,int(4*ss))
            draw.rounded_rectangle(
                (x1-pad+inset,y1-pad+inset,x2+pad-inset,y2+pad-inset),
                radius=max(2,7-j*2),
                outline=(228,231,215,int((72-j*16)*ss)),
                width=1
            )
    font=_font(9)
    draw.rectangle((8,8,min(w-8,190),27),fill=(6,6,5,105))
    draw.text((13,12),"EMULSION / MATERIAL LOSS",font=font,fill=(232,229,212,195))
    return Image.alpha_composite(base,layer).convert("RGB")


def fx_shield(img, people, frame_num, intensity):
    """FARADAY VEIL: diffraction/interference field around detected bodies."""
    arr=np.array(img.convert("RGB"),dtype=np.uint8)
    h,w=arr.shape[:2]
    ss=float(np.clip(float(intensity)/100.0,0.0,1.0))
    safe=_safe_people(people,w,h)
    targets=safe if safe else [(w//2-w//6,h//2-h//4,w//2+w//6,h//2+h//4,.35)]

    xn,yn=_normalized_grids(h,w)
    dx=np.zeros((h,w),np.float32)
    dy=np.zeros((h,w),np.float32)
    phase=frame_num*.055

    for x1,y1,x2,y2,conf in targets:
        px=((x1+x2)/2/w)*2-1
        py=((y1+y2)/2/h)*2-1
        rr=np.sqrt((xn-px)**2+(yn-py)**2)+1e-4
        angular=np.arctan2(yn-py,xn-px)
        wave=np.sin(rr*(28+15*ss)-phase*5+np.sin(angular*3)*.35)
        env=np.exp(-rr*(2.0+.8*(1-ss)))
        field=wave*env*ss*max(.35,conf)
        dx += (xn-px)/rr*field*w*.010
        dy += (yn-py)/rr*field*h*.010

    warped=_warp_image(arr,dx,dy)
    plate=_science_plate_base(Image.fromarray(warped),strength=.36)
    base=Image.fromarray(plate).convert("RGBA")
    layer=Image.new("RGBA",base.size,(0,0,0,0))
    draw=ImageDraw.Draw(layer)

    for x1,y1,x2,y2,conf in targets:
        cx,cy=(x1+x2)//2,(y1+y2)//2
        body=max(18,max(x2-x1,y2-y1)//2)
        for ring in range(7):
            r=body+ring*max(5,int(7+3*ss))
            wob=1.0+.025*math.sin(phase*3+ring*.7)
            box=(cx-r*wob,cy-r*.72*wob,cx+r*wob,cy+r*.72*wob)
            a=int(max(18,105-ring*12)*ss)
            col=(214,234,229,a) if viz_settings.get("overlay_tone")=="bw" else (162,226,218,a)
            start=(frame_num*1.3+ring*17)%360
            draw.arc(box,start=start,end=start+255,fill=col,width=1)

    font=_font(9)
    draw.rectangle((8,8,min(w-8,190),27),fill=(5,8,8,100))
    draw.text((13,12),"FARADAY VEIL / INTERFERENCE",font=font,fill=(220,232,226,200))
    return Image.alpha_composite(base,layer).convert("RGB")


def _expanded_box(x1, y1, x2, y2, w, h, pad=0):
    return (
        max(0, int(x1) - pad),
        max(0, int(y1) - pad),
        min(w, int(x2) + pad),
        min(h, int(y2) + pad),
    )


def _people_mask(size, people, pad=0):
    """Conservative rectangle mask used only to protect the background learner."""
    w, h = size
    mask = np.zeros((h, w), dtype=np.uint8)
    for (x1, y1, x2, y2, _conf) in _safe_people(people, w, h):
        ex1, ey1, ex2, ey2 = _expanded_box(x1, y1, x2, y2, w, h, pad)
        if ex2 > ex1 and ey2 > ey1:
            cv2.rectangle(mask, (ex1, ey1), (ex2, ey2), 255, -1)
    return mask


def _resize_for_alignment(arr):
    """Return a small grayscale registration image and its full/small scale."""
    h, w = arr.shape[:2]
    small_w = min(w, ERASE_ALIGN_WIDTH)
    small_h = max(64, int(round(h * small_w / max(1, w))))
    interp = cv2.INTER_AREA if small_w < w else cv2.INTER_LINEAR
    small = cv2.resize(arr, (small_w, small_h), interpolation=interp)
    if small.ndim == 3:
        small = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
    return cv2.GaussianBlur(small, (5, 5), 0), float(w) / float(small_w)


def _estimate_affine_lite(src_rgb, dst_rgb, exclude_mask=None):
    """Estimate a conservative affine transform mapping src -> dst."""
    src_g, scale = _resize_for_alignment(src_rgb)
    dst_g, _ = _resize_for_alignment(dst_rgb)
    sh, sw = src_g.shape[:2]
    valid = None
    if exclude_mask is not None:
        valid = cv2.resize(exclude_mask, (sw, sh), interpolation=cv2.INTER_NEAREST)
        valid = np.where(valid > 0, 0, 255).astype(np.uint8)
        valid = cv2.erode(valid, np.ones((5, 5), np.uint8), iterations=1)

    orb = cv2.ORB_create(
        nfeatures=ERASE_ALIGN_FEATURES,
        scaleFactor=1.2,
        nlevels=5,
        edgeThreshold=19,
        fastThreshold=14,
    )
    kp1, des1 = orb.detectAndCompute(src_g, valid)
    kp2, des2 = orb.detectAndCompute(dst_g, valid)
    info = {'ok': False, 'status': 'not enough features', 'matches': 0, 'inliers': 0,
            'rotation_deg': 0.0, 'scale': 1.0, 'shift_px': 0.0}
    matrix_small = None

    if des1 is not None and des2 is not None and len(kp1) >= 8 and len(kp2) >= 8:
        matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(des1, des2)
        matches = sorted(matches, key=lambda m: m.distance)
        if matches:
            cutoff = max(32.0, float(np.median([m.distance for m in matches])) * 1.65)
            good = [m for m in matches if m.distance <= cutoff][:90]
        else:
            good = []
        info['matches'] = len(good)
        if len(good) >= 8:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            matrix_small, inlier_mask = cv2.estimateAffinePartial2D(
                src_pts, dst_pts, method=cv2.RANSAC,
                ransacReprojThreshold=2.6, maxIters=700,
                confidence=0.97, refineIters=5,
            )
            info['inliers'] = int(inlier_mask.sum()) if inlier_mask is not None else 0

    # Low-texture fallback: translation-only phase correlation.
    if matrix_small is None or info['inliers'] < ERASE_ALIGN_MIN_INLIERS:
        try:
            shift, response = cv2.phaseCorrelate(src_g.astype(np.float32), dst_g.astype(np.float32))
            if response >= 0.08:
                matrix_small = np.array([[1.0, 0.0, shift[0]], [0.0, 1.0, shift[1]]], dtype=np.float32)
                info['status'] = f'phase {response:.2f}'
                info['inliers'] = max(info['inliers'], ERASE_ALIGN_MIN_INLIERS)
        except Exception:
            matrix_small = None

    if matrix_small is None:
        return None, info

    a, b, tx = map(float, matrix_small[0])
    c, d, ty = map(float, matrix_small[1])
    scale_est = math.sqrt(max(1e-8, a*a + c*c))
    rotation = math.degrees(math.atan2(c, a))
    shift_px = math.hypot(tx * scale, ty * scale)
    max_shift = math.hypot(dst_rgb.shape[1], dst_rgb.shape[0]) * ERASE_ALIGN_MAX_SHIFT_RATIO
    valid_transform = (
        ERASE_ALIGN_MIN_SCALE <= scale_est <= ERASE_ALIGN_MAX_SCALE and
        abs(rotation) <= ERASE_ALIGN_MAX_ROT_DEG and
        shift_px <= max_shift and
        info['inliers'] >= ERASE_ALIGN_MIN_INLIERS
    )
    info.update({
        'ok': bool(valid_transform),
        'status': 'aligned' if valid_transform else 'motion outside tolerance',
        'rotation_deg': round(rotation, 2),
        'scale': round(scale_est, 4),
        'shift_px': round(shift_px, 1),
    })
    if not valid_transform:
        return None, info

    full = np.array(matrix_small, dtype=np.float32)
    full[0, 2] *= scale
    full[1, 2] *= scale
    return full, info


def _align_clean_plate(bg, current, people, cache_key='mapped', force=False):
    """Warp a clean plate to the live frame with cached, low-rate registration."""
    global _alignment_status
    h, w = current.shape[:2]
    if bg.shape[:2] != (h, w):
        interpolation = cv2.INTER_CUBIC if w*h > bg.shape[1]*bg.shape[0] else cv2.INTER_AREA
        bg = cv2.resize(bg, (w, h), interpolation=interpolation)

    key = (str(cache_key), w, h)
    with _alignment_lock:
        cached = dict(_alignment_cache.get(key, {}))
    due = force or not cached or (_frame_count - int(cached.get('frame', -999))) >= ERASE_ALIGN_EVERY

    matrix = cached.get('matrix')
    info = cached.get('info', {'ok': False, 'status': 'identity'})
    if due:
        exclude = _people_mask((w, h), people, pad=ERASE_PAD + 18) if people else None
        candidate, candidate_info = _estimate_affine_lite(bg, current, exclude_mask=exclude)
        if candidate is not None:
            matrix = candidate
            info = candidate_info
        elif matrix is None:
            matrix = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
            info = candidate_info
        else:
            # Keep the previous good transform across a brief registration miss.
            info = dict(candidate_info)
            info['status'] = 'holding last alignment'
        with _alignment_lock:
            _alignment_cache[key] = {'matrix': matrix.copy(), 'info': dict(info), 'frame': _frame_count}

    aligned = cv2.warpAffine(
        bg, matrix, (w, h), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )
    status = dict(info)
    status['source'] = str(cache_key)
    with _alignment_lock:
        _alignment_status = status
    return aligned, status


def _load_mapped_background():
    global _mapped_background, _mapped_created_at
    try:
        if not os.path.exists(MAPPED_BACKGROUND_PATH):
            return
        mapped = np.array(Image.open(MAPPED_BACKGROUND_PATH).convert('RGB'))
        if mapped.ndim != 3 or mapped.shape[2] != 3:
            raise RuntimeError('scene map is not an RGB image')
        with _scene_map_lock:
            _mapped_background = mapped
            _mapped_created_at = datetime.fromtimestamp(
                os.path.getmtime(MAPPED_BACKGROUND_PATH)
            ).isoformat(timespec='seconds')
            _scene_scan_state.update({
                'phase': 'ready',
                'status': f'mapped scene ready {mapped.shape[1]}x{mapped.shape[0]}',
                'percent': 100,
                'alignment': 'handheld compensation enabled',
            })
        print(f"✓ Loaded erasure scene map: {MAPPED_BACKGROUND_PATH}")
    except Exception as e:
        print(f"Scene map load failed: {e}")


def start_scene_scan(target_frames=None):
    """Begin a registered, multi-frame empty-scene scan for Mapped Erase."""
    global _scene_scan_sum, _scene_scan_prev, _scene_scan_reference
    target = int(np.clip(
        target_frames if target_frames is not None else SCENE_SCAN_TARGET,
        8, 60
    ))
    with _scene_map_lock:
        _scene_scan_sum = None
        _scene_scan_prev = None
        _scene_scan_reference = None
        _scene_scan_state.update({
            'active': True,
            'phase': 'waiting',
            'status': 'clear all people from the frame',
            'accepted': 0,
            'rejected': 0,
            'target': target,
            'percent': 0,
            'alignment': 'waiting for first clean frame',
            'last_motion': None,
            'started_at': datetime.now().isoformat(timespec='seconds'),
        })
    return dict(_scene_scan_state)


def clear_scene_map():
    global _mapped_background, _mapped_created_at
    global _scene_scan_sum, _scene_scan_prev, _scene_scan_reference
    with _scene_map_lock:
        _mapped_background = None
        _mapped_created_at = None
        _scene_scan_sum = None
        _scene_scan_prev = None
        _scene_scan_reference = None
        _scene_scan_state.update({
            'active': False,
            'phase': 'idle',
            'status': 'not mapped',
            'accepted': 0,
            'rejected': 0,
            'target': SCENE_SCAN_TARGET,
            'percent': 0,
            'alignment': 'idle',
            'last_motion': None,
            'started_at': None,
        })
    with _alignment_lock:
        _alignment_cache.clear()
    try:
        if os.path.exists(MAPPED_BACKGROUND_PATH):
            os.remove(MAPPED_BACKGROUND_PATH)
    except Exception as e:
        print(f"Could not remove old scene map: {e}")


def _finalize_scene_map(preview_mapped, accepted):
    """Capture/average a registered high-resolution plate without freezing Flask."""
    global _mapped_background, _mapped_created_at
    global _background_model, _background_frames, _background_empty_frames, _background_last_update

    mapped = preview_mapped
    try:
        samples = []
        reference = None
        for idx in range(SCENE_MAP_HIRES_SAMPLES):
            with _scene_map_lock:
                _scene_scan_state['status'] = f'finalizing high-res plate {idx+1}/{SCENE_MAP_HIRES_SAMPLES}'
                _scene_scan_state['percent'] = 92 + int(7 * idx / max(1, SCENE_MAP_HIRES_SAMPLES))
            main_frame = _capture_stream_array('main')
            main_arr = np.array(prepare_camera_frame(main_frame), dtype=np.uint8)
            if reference is None:
                reference = main_arr
                samples.append(main_arr.astype(np.float32))
            else:
                matrix, info = _estimate_affine_lite(main_arr, reference)
                if matrix is not None:
                    aligned = cv2.warpAffine(main_arr, matrix, (reference.shape[1], reference.shape[0]),
                                             flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
                    samples.append(aligned.astype(np.float32))
                else:
                    # A failed burst sample is skipped rather than blurring the map.
                    print(f"Scene map burst sample skipped: {info.get('status')}")
            time.sleep(0.025)
        if samples:
            mapped = np.clip(np.mean(samples, axis=0), 0, 255).astype(np.uint8)
    except Exception as e:
        print(f"High-resolution scene-map burst failed; using preview map: {e}")

    with _scene_map_lock:
        _mapped_background = mapped
        _mapped_created_at = datetime.now().isoformat(timespec='seconds')
        mh, mw = mapped.shape[:2]
        _scene_scan_state.update({
            'active': False,
            'phase': 'ready',
            'status': f'ready — {mw}x{mh} clean plate saved',
            'percent': 100,
            'alignment': 'handheld compensation enabled',
        })

    try:
        tmp_path = MAPPED_BACKGROUND_PATH + '.tmp.png'
        Image.fromarray(mapped).save(tmp_path, 'PNG', compress_level=2)
        os.replace(tmp_path, MAPPED_BACKGROUND_PATH)
        print(f"✓ Scene map captured at {mapped.shape[1]}x{mapped.shape[0]}: {MAPPED_BACKGROUND_PATH}")
    except Exception as e:
        print(f"Scene map save failed: {e}")

    with _background_lock:
        _background_model = preview_mapped.astype(np.float32)
        _background_frames = max(_background_frames, accepted)
        _background_empty_frames = max(_background_empty_frames, accepted)
        _background_last_update = time.monotonic()


def update_scene_scan(img, people):
    """Build a clean plate while tolerating small handheld translation/rotation."""
    global _scene_scan_sum, _scene_scan_prev, _scene_scan_reference
    global _scene_scan_finalize_thread

    with _scene_map_lock:
        if not _scene_scan_state['active']:
            return

    arr = np.array(img.convert('RGB'), dtype=np.uint8)
    if people:
        with _scene_map_lock:
            _scene_scan_state['rejected'] += 1
            _scene_scan_state['phase'] = 'waiting'
            _scene_scan_state['status'] = 'person detected — clear the frame'
            _scene_scan_state['alignment'] = 'paused'
        return

    aligned = arr
    with _scene_map_lock:
        reference = None if _scene_scan_reference is None else _scene_scan_reference.copy()

    if reference is None:
        with _scene_map_lock:
            _scene_scan_reference = arr.copy()
            _scene_scan_prev = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
            _scene_scan_sum = arr.astype(np.float64)
            _scene_scan_state['accepted'] = 1
            _scene_scan_state['phase'] = 'scanning'
            _scene_scan_state['status'] = f'scanning 1/{_scene_scan_state["target"]}'
            _scene_scan_state['alignment'] = 'reference locked'
            _scene_scan_state['percent'] = int(90 / max(1, _scene_scan_state['target']))
        return

    matrix, info = _estimate_affine_lite(arr, reference)
    if matrix is None:
        with _scene_map_lock:
            _scene_scan_state['rejected'] += 1
            _scene_scan_state['status'] = 'camera moved beyond handheld tolerance'
            _scene_scan_state['alignment'] = info.get('status', 'alignment failed')
        return

    aligned = cv2.warpAffine(arr, matrix, (reference.shape[1], reference.shape[0]),
                             flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
    gray = cv2.cvtColor(aligned, cv2.COLOR_RGB2GRAY)
    ref_gray = cv2.cvtColor(reference, cv2.COLOR_RGB2GRAY)
    residual = float(np.mean(cv2.absdiff(gray, ref_gray)))
    if residual > SCENE_SCAN_MOTION_LIMIT:
        with _scene_map_lock:
            _scene_scan_state['rejected'] += 1
            _scene_scan_state['status'] = f'lighting/motion changed too much ({residual:.1f})'
            _scene_scan_state['alignment'] = info.get('status', 'aligned')
            _scene_scan_state['last_motion'] = round(residual, 1)
        return

    preview_mapped = None
    accepted = 0
    with _scene_map_lock:
        if _scene_scan_sum is None or _scene_scan_sum.shape != aligned.shape:
            _scene_scan_sum = np.zeros(aligned.shape, dtype=np.float64)
        _scene_scan_sum += aligned.astype(np.float64)
        _scene_scan_prev = gray
        _scene_scan_state['accepted'] += 1
        accepted = _scene_scan_state['accepted']
        target = _scene_scan_state['target']
        _scene_scan_state['phase'] = 'scanning'
        _scene_scan_state['status'] = f'scanning {accepted}/{target}'
        _scene_scan_state['alignment'] = (
            f"shift {info.get('shift_px', 0):.0f}px / rot {info.get('rotation_deg', 0):.1f}°"
        )
        _scene_scan_state['last_motion'] = round(residual, 1)
        _scene_scan_state['percent'] = min(90, int(90 * accepted / max(1, target)))
        if accepted < target:
            return

        preview_mapped = np.clip(_scene_scan_sum / max(1, accepted), 0, 255).astype(np.uint8)
        _scene_scan_state.update({
            'active': False,
            'phase': 'finalizing',
            'status': 'preview map complete — finalizing high-res plate',
            'percent': 92,
        })
        _scene_scan_sum = None
        _scene_scan_prev = None
        _scene_scan_reference = None

    _scene_scan_finalize_thread = threading.Thread(
        target=_finalize_scene_map, args=(preview_mapped, accepted), daemon=True
    )
    _scene_scan_finalize_thread.start()


def update_background_model(img, people):
    """Maintain a clean plate in the current handheld camera coordinate frame."""
    global _background_model, _background_frames, _background_empty_frames, _background_last_update
    arr_u8 = np.array(img.convert('RGB'), dtype=np.uint8)
    arr = arr_u8.astype(np.float32)
    h, w = arr.shape[:2]
    people = _safe_people(people, w, h)

    with _background_lock:
        existing = None if _background_model is None else np.clip(_background_model, 0, 255).astype(np.uint8)

    if existing is None or existing.shape != arr_u8.shape:
        if people:
            return
        with _background_lock:
            _background_model = arr.copy()
            _background_frames = 1
            _background_empty_frames = 1
            _background_last_update = time.monotonic()
        return

    aligned, align_info = _align_clean_plate(existing, arr_u8, people, cache_key='adaptive', force=True)
    # When a person is present, never let a failed large camera move contaminate
    # the clean plate. An empty frame may safely become a new reference.
    if people and not align_info.get('ok', False) and align_info.get('status') != 'holding last alignment':
        return
    if (not people) and not align_info.get('ok', False):
        # With an empty frame, a large reframe can safely establish a fresh
        # adaptive reference instead of repeatedly warping/smearing the old one.
        working = arr.copy()
    else:
        working = aligned.astype(np.float32)

    if people:
        keep = (_people_mask((w, h), people, pad=ERASE_PAD + 20) == 0)
        lr = ERASE_BG_LR
    else:
        keep = np.ones((h, w), dtype=bool)
        lr = ERASE_EMPTY_LR

    working[keep] = working[keep] * (1.0 - lr) + arr[keep] * lr
    with _background_lock:
        _background_model = working
        _background_frames += 1
        if not people:
            _background_empty_frames += 1
        _background_last_update = time.monotonic()


def get_background_image(size):
    """Return adaptive clean plate, resizing it for high-resolution stills."""
    w, h = size
    with _background_lock:
        if _background_model is None:
            return None, 0, 0
        bg = np.clip(_background_model, 0, 255).astype(np.uint8)
        frames = _background_frames
        empty_frames = _background_empty_frames
    if bg.shape[:2] != (h, w):
        interpolation = cv2.INTER_CUBIC if (w*h) > (bg.shape[1]*bg.shape[0]) else cv2.INTER_AREA
        bg = cv2.resize(bg, (w, h), interpolation=interpolation)
    return bg, frames, empty_frames


def get_mapped_background(size):
    w, h = size
    with _scene_map_lock:
        if _mapped_background is None:
            return None
        mapped = _mapped_background.copy()
    if mapped.shape[:2] != (h, w):
        interpolation = cv2.INTER_CUBIC if (w*h) > (mapped.shape[1]*mapped.shape[0]) else cv2.INTER_AREA
        mapped = cv2.resize(mapped, (w, h), interpolation=interpolation)
    return mapped


def get_erasure_background(size, source='adaptive'):
    if source == 'mapped':
        mapped = get_mapped_background(size)
        return mapped, mapped is not None, 'MAPPED SCENE'
    bg, _frames, empty_frames = get_background_image(size)
    ready = bg is not None and empty_frames >= ERASE_BG_READY_EMPTY_FRAMES
    return bg, ready, 'ADAPTIVE CLEAN PLATE'


def _person_shape_prior(box_w, box_h):
    """Small core silhouette that keeps removal solid when clothing matches the wall."""
    prior = np.zeros((box_h, box_w), dtype=np.uint8)
    if box_w < 4 or box_h < 8:
        return prior
    cx = box_w // 2
    head_r = max(2, min(box_w // 5, box_h // 10))
    head_y = max(head_r + 1, int(box_h * 0.13))
    cv2.circle(prior, (cx, head_y), head_r, 255, -1)
    shoulder_y = int(box_h * 0.22)
    hip_y = int(box_h * 0.62)
    half_shoulder = max(2, int(box_w * 0.28))
    half_hip = max(2, int(box_w * 0.18))
    torso = np.array([
        [cx-half_shoulder, shoulder_y], [cx+half_shoulder, shoulder_y],
        [cx+half_hip, hip_y], [cx-half_hip, hip_y]
    ], dtype=np.int32)
    cv2.fillConvexPoly(prior, torso, 255)
    leg_w = max(2, int(box_w * 0.12))
    cv2.rectangle(prior, (cx-half_hip, hip_y), (cx-1, box_h-1), 255, -1)
    cv2.rectangle(prior, (cx+1, hip_y), (cx+half_hip, box_h-1), 255, -1)
    prior = cv2.dilate(prior, np.ones((max(3, leg_w//2*2+1), 3), np.uint8), iterations=1)
    return prior


def _foreground_person_mask(arr, bg, people, pad=ERASE_PAD):
    """Segment people by comparing the current frame with the clean plate."""
    global _erasure_mask_shape
    h, w = arr.shape[:2]
    with _erasure_mask_lock:
        if _erasure_mask_shape != (h, w):
            _erasure_mask_history.clear()
            _erasure_mask_shape = (h, w)
    current_gray = cv2.GaussianBlur(cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY), (5, 5), 0)
    bg_gray = cv2.GaussianBlur(cv2.cvtColor(bg, cv2.COLOR_RGB2GRAY), (5, 5), 0)
    full_mask = np.zeros((h, w), dtype=np.uint8)

    for x1, y1, x2, y2, _conf in _safe_people(people, w, h):
        ex1, ey1, ex2, ey2 = _expanded_box(x1, y1, x2, y2, w, h, pad)
        if ex2 <= ex1 or ey2 <= ey1:
            continue
        diff = cv2.absdiff(
            current_gray[ey1:ey2, ex1:ex2],
            bg_gray[ey1:ey2, ex1:ex2]
        )
        if diff.size == 0:
            continue
        otsu_value, _ = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        threshold = int(np.clip(max(ERASE_DIFF_MIN, otsu_value * 0.62), ERASE_DIFF_MIN, 42))
        local = np.where(diff >= threshold, 255, 0).astype(np.uint8)
        local = cv2.morphologyEx(local, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8), iterations=2)
        local = cv2.morphologyEx(local, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)

        # Keep changed components plus a conservative body core.  This is much
        # tighter than replacing the full YOLO rectangle, but avoids holes when
        # a shirt is visually similar to the background.
        prior = _person_shape_prior(ex2-ex1, ey2-ey1)
        # The silhouette prior may fill clothing-colored holes, but only near
        # pixels that actually differ from the registered clean plate. This
        # avoids erasing static furniture merely because it lies inside a box.
        support = cv2.dilate(local, np.ones((13, 13), np.uint8), iterations=1)
        local = cv2.max(local, cv2.bitwise_and(prior, support))
        local = cv2.dilate(local, np.ones((5, 5), np.uint8), iterations=1)
        full_mask[ey1:ey2, ex1:ex2] = cv2.max(full_mask[ey1:ey2, ex1:ex2], local)

    with _erasure_mask_lock:
        if full_mask.max() > 0:
            _erasure_mask_history.append(full_mask.copy())
        elif people:
            _erasure_mask_history.append(full_mask.copy())
        if _erasure_mask_history:
            stable = np.maximum.reduce(list(_erasure_mask_history))
        else:
            stable = full_mask
    return stable


def _match_clean_plate_lighting(arr, bg, mask):
    """Match mapped/adaptive plate brightness using pixels outside erased people."""
    sample = (mask[::6, ::6] == 0)
    if np.count_nonzero(sample) < 300:
        return bg
    cur = arr[::6, ::6, 0][sample].astype(np.float32)
    ref = bg[::6, ::6, 0][sample].astype(np.float32)
    cur_mean, ref_mean = float(cur.mean()), float(ref.mean())
    cur_std, ref_std = float(cur.std()), float(ref.std())
    gain = np.clip(cur_std / max(3.0, ref_std), 0.88, 1.12)
    offset = np.clip(cur_mean - ref_mean * gain, -22.0, 22.0)
    return np.clip(bg.astype(np.float32) * gain + offset, 0, 255).astype(np.uint8)


def _replace_with_clean_plate(arr, bg, mask):
    """Composite true clean-plate pixels with only a narrow anti-aliased seam."""
    if mask.max() == 0:
        return arr
    matched_bg = _match_clean_plate_lighting(arr, bg, mask)
    hard = mask.astype(np.float32) / 255.0
    if ERASE_FEATHER > 0:
        soft = cv2.GaussianBlur(hard, (0, 0), ERASE_FEATHER)
        alpha = np.maximum(hard, soft * 0.88)
    else:
        alpha = hard
    alpha = np.clip(alpha, 0.0, 1.0)[:, :, None]
    return np.clip(
        arr.astype(np.float32) * (1.0 - alpha) + matched_bg.astype(np.float32) * alpha,
        0, 255
    ).astype(np.uint8)


def fx_erased(img, people, frame_num, intensity, source='adaptive'):
    """Replace tracked people with real pixels from an empty-scene clean plate."""
    arr = np.array(img.convert('RGB'))
    h, w = arr.shape[:2]
    s = intensity / 100.0
    people = _safe_people(people, w, h)
    font = _font(10)
    font_big = _font(12)

    bg, bg_ready, source_label = get_erasure_background((w, h), source=source)
    alignment_note = 'adaptive tracking'
    if bg_ready and bg is not None and source == 'mapped':
        bg, align_info = _align_clean_plate(bg, arr, people, cache_key='mapped')
        alignment_note = align_info.get('status', 'alignment unknown')
    label_state = 'CLEAN PLATE REQUIRED'
    replaced = False
    erase_mask = np.zeros((h, w), dtype=np.uint8)

    if not people:
        with _erasure_mask_lock:
            _erasure_mask_history.clear()

    if people and bg_ready and bg is not None:
        erase_mask = _foreground_person_mask(arr, bg, people, pad=ERASE_PAD)
        arr = _replace_with_clean_plate(arr, bg, erase_mask)
        label_state = source_label
        replaced = erase_mask.max() > 0
    elif people:
        # Deliberately leave the original pixels untouched until a clean plate
        # exists.  Inpainting large people produces the smeared/blurred result
        # this version is designed to avoid.
        label_state = 'WAITING FOR EMPTY SCENE'

    result = Image.fromarray(arr).convert('RGBA')
    overlay = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)

    if people:
        for idx, (x1, y1, x2, y2, conf) in enumerate(people):
            ex1, ey1, ex2, ey2 = _expanded_box(x1, y1, x2, y2, w, h, 0)
            alpha = int(235 * max(0.35, s))
            green = (0, 255, 85, alpha)
            bright = (0, 255, 135, min(255, alpha + 10))
            width = max(1, int(1 + 3 * s))
            draw.rectangle([(ex1, ey1), (ex2, ey2)], outline=green, width=width)
            cl = min(30, max(10, (ex2 - ex1) // 4), max(10, (ey2 - ey1) // 4))
            for sx, sy in [(ex1, ey1), (ex2, ey1), (ex1, ey2), (ex2, ey2)]:
                xdir = 1 if sx == ex1 else -1
                ydir = 1 if sy == ey1 else -1
                draw.line([(sx, sy), (sx + xdir * cl, sy)], fill=bright, width=width + 1)
                draw.line([(sx, sy), (sx, sy + ydir * cl)], fill=bright, width=width + 1)
            tag = f"REMOVED {idx+1}  {conf:.0%}  {label_state}"
            _glow_text(draw, (ex1, max(0, ey1 - 17)), tag, font, fill=(0,255,100,int(215*max(.35,s))), radius=1)
    else:
        ready = 'READY' if bg_ready else 'NOT READY'
        if source == 'mapped':
            with _scene_map_lock:
                scan_status = _scene_scan_state['status']
                accepted = _scene_scan_state['accepted']
                target = _scene_scan_state['target']
            status_text = f"SCENE MAP {ready}  {scan_status}  {accepted}/{target}"
        else:
            _bg, _frames, empty_frames = get_background_image((w, h))
            status_text = f"ADAPTIVE PLATE {ready}  clean frames {empty_frames}/{ERASE_BG_READY_EMPTY_FRAMES}"
        _glow_text(draw, (16, h - 70), status_text, font_big, fill=(0,255,90,int(165*max(.35,s))), radius=1)

    if source == 'mapped':
        title = 'MAPPED ERASE'
        subtitle = (f'tracked foreground → registered scene map ({alignment_note})'
                    if replaced else 'scan an empty scene; small handheld motion is allowed')
    else:
        title = 'ERASED'
        subtitle = 'tracked foreground → adaptive clean-plate pixels' if replaced else 'learning the empty scene / no synthetic blur'
    _mode_stamp(overlay, title, subtitle, frame_num, max(.25, s), accent=(0,255,85))
    return Image.alpha_composite(result, overlay).convert('RGB')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  RAW CAMERA MODE
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def fx_raw(img):
    """Clean camera feed — no overlays. Camera controls applied via /cam_controls."""
    return img

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  4D HYPERSPACE MODE
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _hyperspace_field(xn, yn, t, w_axis, submode, d4, s):
    """Return dx/dy fields for a synthetic W-slice of the 2D camera image.

    A Raspberry Pi camera cannot directly sample a real 4D world. This engine
    treats the live camera image as a 3D membrane and renders multiple animated
    cross-sections along a synthetic fourth spatial axis W. The output is a
    cinematic approximation of 'seeing through' ordinary space into hyperspace.
    """
    r2 = xn * xn + yn * yn
    r = np.sqrt(r2) + 1e-5
    a = np.arctan2(yn, xn)

    # 4D perspective factor: surfaces nearer along W appear stretched; farther
    # W-slices compress.  d4 comes from the UI depth slider.
    f4 = d4 / np.clip(d4 - w_axis - 0.35 * np.sin(r * math.pi * 2.0 + t), 0.22, None)
    hyper = np.sin((r * 5.0 + w_axis * 2.4 - t * 1.25) * math.pi)
    fold = np.cos(a * 4.0 + t * 0.7 + w_axis * 3.0)

    dx = (xn * (f4 - 1.0) + yn * hyper * 0.05 + np.cos(a * 3.0 + t) * fold * 0.018) * s
    dy = (yn * (f4 - 1.0) - xn * hyper * 0.05 + np.sin(a * 5.0 - t) * fold * 0.018) * s

    if submode in ('tesseract', 'all4d'):
        dx += np.sign(np.sin((xn + w_axis) * math.pi * 2 + t)) * 0.018 * s
        dy += np.sign(np.sin((yn - w_axis) * math.pi * 2 - t * 0.8)) * 0.018 * s
    if submode in ('clifford', 'all4d'):
        theta = xn * math.pi * 2.0 + t + w_axis
        phi = yn * math.pi * 2.0 - t * 0.7 + w_axis * 1.7
        dx += (np.cos(theta) * np.sin(phi + t) * 0.065) * s
        dy += (np.sin(theta + phi) * 0.055) * s
    if submode in ('lissajous', 'all4d'):
        p = xn * 2.5 + yn * 1.7 + w_axis + t
        dx += np.cos(p * 3.0) * np.sin(p * 5.0 + t) * 0.07 * s
        dy += np.sin(p * 4.0 - t) * np.cos(p * 2.0) * 0.07 * s
    return dx, dy, f4


def fx_hyperspace(img, people, frame_num, intensity):
    """4D vision engine: multi-W-slice image transformation + projected geometry."""
    submode = hyperspace_settings.get('submode', 'tesseract')
    rot_speed = float(hyperspace_settings.get('rot_speed', 1.0))
    d4 = float(hyperspace_settings.get('w_depth', 2.5))
    w, h = img.size
    cx, cy = w // 2, h // 2
    s = intensity / 100.0
    t = frame_num * 0.022 * rot_speed
    font = _font(9)
    arr = np.array(img.convert('RGB'), dtype=np.uint8)
    xn, yn = _normalized_grids(h, w)

    # Render several W-slices.  These are not overlays; each slice remaps the
    # live camera pixels as if nearby 4D cross-sections were visible at once.
    slice_ws = [-0.72, 0.0, 0.72] if submode != 'all4d' else [-1.0, -0.45, 0.0, 0.45, 1.0]
    accum = np.zeros_like(arr, dtype=np.float32)
    weight_sum = 0.0
    all_depth = []
    for i, w_axis in enumerate(slice_ws):
        dxn, dyn, f4 = _hyperspace_field(xn, yn, t + i * 0.13, w_axis, submode, d4, s)
        dx = dxn * w
        dy = dyn * h
        sample = _warp_image(arr, dx, dy)
        hue = (0.58 + 0.10 * w_axis + 0.05 * math.sin(t + i)) % 1.0
        tint = np.array(colorsys.hsv_to_rgb(hue, 0.72, 1.0), dtype=np.float32) * (18 + 24 * abs(w_axis)) * s
        sample = np.clip(sample.astype(np.float32) + tint[None, None, :], 0, 255)
        wt = 1.0 / (1.0 + abs(w_axis) * 0.8)
        # Make side W-slices translucent; center slice stays anchored to reality.
        accum += sample * wt
        weight_sum += wt
        all_depth.append(f4)
    warped = np.clip(accum / max(0.001, weight_sum), 0, 255).astype(np.uint8)

    # Extra spatial-axis chromatic split: red and blue do not share the same W.
    dxr, dyr, _ = _hyperspace_field(xn, yn, t + 0.31, 0.92, submode, d4, s * 0.45)
    dxb, dyb, _ = _hyperspace_field(xn, yn, t - 0.23, -0.92, submode, d4, s * 0.45)
    warped[:, :, 0] = _warp_channel(arr[:, :, 0], dxr * w, dyr * h)
    warped[:, :, 2] = _warp_channel(arr[:, :, 2], dxb * w, dyb * h)

    # Hypersphere body lensing: detected bodies are treated as 4D objects whose
    # visible cross-section changes with W.  If no person is detected, one
    # ambient hypersphere keeps the mode visually legible.
    targets = _safe_people(people, w, h)
    if submode in ('hyperspheres', 'all4d') or targets:
        if not targets:
            targets = [(int(w*0.20), int(h*0.24), int(w*0.80), int(h*0.76), 0.45)]
        dx = np.zeros((h, w), dtype=np.float32)
        dy = np.zeros((h, w), dtype=np.float32)
        for pi, (x1, y1, x2, y2, conf) in enumerate(targets):
            px = ((x1 + x2) / 2 / w) * 2 - 1
            py = ((y1 + y2) / 2 / h) * 2 - 1
            radius = max(0.08, ((x2-x1 + y2-y1) * 0.30) / min(w, h))
            w_cross = math.sin(t * 1.15 + pi * 1.9)
            cross = max(0.08, radius * math.sqrt(max(0.02, 1.0 - w_cross*w_cross)))
            dist2 = (xn - px)**2 + (yn - py)**2 + 1e-6
            lens = np.exp(-dist2 / (2 * (cross * 0.92)**2)) * conf * s
            dx -= (xn - px) * lens * w * 0.095
            dy -= (yn - py) * lens * h * 0.095
            # Frame-drag swirl.
            dx -= (yn - py) * lens * w * 0.035
            dy += (xn - px) * lens * h * 0.035
        warped = _warp_image(warped, dx, dy)

    # Blend with reality so low intensity remains usable.
    if s < 0.98:
        warped = cv2.addWeighted(warped, max(0.08, s), arr, 1.0 - max(0.08, s), 0)
    warped = _vignette_rgb(_chromatic_aberration(warped, frame_num, int(2 + 5*s)), strength=0.25 * s)

    img_out = Image.fromarray(warped).convert('RGBA')
    layer = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)

    # 4D geometry as instrument overlay, not the main effect.
    R = (rot4d((0, 3), t * 0.72) @ rot4d((1, 3), t * 0.48) @ rot4d((2, 3), t * 0.33) @ rot4d((0, 1), t * 0.25))
    hue_t = (0.62 + 0.14 * math.sin(t * 0.4)) % 1.0
    if submode in ('tesseract', 'all4d') and s > 0.15:
        scale = min(w, h) * 0.19
        verts = (_TESS_VERTS @ R.T) * 0.86
        v2d, depth = proj4d(verts, d4=d4)
        v2d = v2d * scale + np.array([cx, cy])
        for i, j in _TESS_EDGES:
            dd = float((depth[i] + depth[j]) * 0.5)
            alpha = int(np.clip(dd * 74 * s, 12, 130))
            r, g, b = [int(c * 255) for c in colorsys.hsv_to_rgb((hue_t + dd * 0.08) % 1.0, 0.7, 1.0)]
            draw.line([(int(v2d[i,0]), int(v2d[i,1])), (int(v2d[j,0]), int(v2d[j,1]))], fill=(r,g,b,alpha), width=1)
    if submode in ('clifford', 'all4d') and s > 0.15:
        # Draw a sparse projected Clifford torus meridian set.
        for k in range(0, _N_CLIFF, 4):
            pts = []
            idxs = [k * _N_CLIFF + j for j in range(_N_CLIFF)]
            verts = (_CLIFF_VERTS[idxs] @ R.T)
            v2d, depth = proj4d(verts, d4=d4)
            v2d = v2d * (min(w,h)*0.24) + np.array([cx, cy])
            for p in v2d:
                pts.append((int(p[0]), int(p[1])))
            if len(pts) > 2:
                draw.line(pts + [pts[0]], fill=(160, 110, 255, int(45*s)), width=1)
    if submode in ('hyperspheres', 'all4d'):
        hs_targets = _safe_people(people, w, h) or [(int(w*0.20), int(h*0.24), int(w*0.80), int(h*0.76), 0.45)]
        for pi, (x1, y1, x2, y2, conf) in enumerate(hs_targets):
            ccx, ccy = (x1+x2)//2, (y1+y2)//2
            r4 = (x2-x1 + y2-y1) * 0.32
            wc = math.sin(t * 1.15 + pi * 1.9)
            cross = int(max(8, r4 * math.sqrt(max(0.02, 1 - wc*wc))))
            alpha = int(150 * s * (1 - abs(wc)) * max(0.35, conf))
            draw.ellipse([(ccx-cross, ccy-cross//2), (ccx+cross, ccy+cross//2)], outline=(110, 240, 255, alpha), width=2)
            draw.text((x1, max(0, y1 - 17)), f"W-CROSS {wc:+.2f}", fill=(180,255,230,int(145*s)), font=font)

    sub_lbl = {'tesseract': 'TESSERACT LATTICE', 'clifford': 'CLIFFORD TORUS', 'lissajous': 'LISSAJOUS FIELD', 'hyperspheres': 'HYPERSPHERES', 'all4d': 'ALL W-SLICES'}
    info = [
        f"4D VISION: {sub_lbl.get(submode, submode.upper())}",
        f"W-SLICES: {len(slice_ws)}   DEPTH: {d4:.1f}",
        f"ROTATION: {(t % (2*math.pi)):.2f} rad",
    ]
    box_w = 205
    draw.rounded_rectangle([(w - box_w - 10, h - 64), (w - 10, h - 10)], radius=7, fill=(5, 5, 18, int(128*s)), outline=(160,120,255,int(70*s)), width=1)
    for li, line in enumerate(info):
        draw.text((w - box_w, h - 56 + li*15), line, fill=(165,210,255,int(190*s)), font=font)
    _mode_stamp(layer, "4D HYPERSPACE", "multi-slice W-axis camera transform", frame_num, s, accent=(170,130,255))
    return Image.alpha_composite(img_out, layer).convert('RGB')


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  OFFICIAL-DATA ENVIRONMENT VISUALIZATION MODES
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _env_palette(alpha=220):
    if viz_settings.get('overlay_tone','color') == 'bw':
        return {'field':(245,245,245,alpha),'field2':(150,150,150,alpha),'particle':(225,225,225,alpha),'accent':(110,110,110,alpha),'panel':(8,8,8,min(210,alpha))}
    return {'field':(50,235,255,alpha),'field2':(165,100,255,alpha),'particle':(255,188,55,alpha),'accent':(255,75,120,alpha),'panel':(3,9,18,min(200,alpha))}


def _env_drives(snapshot):
    sw=snapshot.get('swpc',{}); gm=snapshot.get('geomag',{}); aur=snapshot.get('aurora',{}); cr=snapshot.get('cosmic_ray',{})
    kp=_float_or_none(sw.get('kp')) or 0.0; speed=_float_or_none(sw.get('solar_wind_speed')) or 350.0; bz=_float_or_none(sw.get('bz_gsm')) or 0.0
    proton=max(0.0,_float_or_none(sw.get('proton_flux_10mev')) or 0.0); electron=max(0.0,_float_or_none(sw.get('electron_flux_2mev')) or 0.0)
    df=abs(_float_or_none(gm.get('df')) or 0.0); aurora=max(0.0,_float_or_none(aur.get('probability')) or 0.0); cosmic=abs(_float_or_none(cr.get('relative_percent')) or 0.0)
    kp_n=float(np.clip(kp/9,0,1)); speed_n=float(np.clip((speed-250)/650,0,1)); south=float(np.clip(max(0,-bz)/20,0,1)); delta=float(np.clip(df/250,0,1))
    pn=float(np.clip(math.log10(1+proton)/3,0,1)); en=float(np.clip(math.log10(1+electron)/6,0,1)); an=float(np.clip(aurora/100,0,1)); cn=float(np.clip(cosmic/5,0,1))
    magnetic=float(np.clip(.32*kp_n+.23*south+.30*delta+.15*an,0,1)); particles=float(np.clip(.40*pn+.15*en+.18*speed_n+.17*cn+.10*an,0,1))
    return magnetic,particles


def _env_field_angle(snapshot):
    d=_float_or_none(snapshot.get('wmm',{}).get('d')) or 0.0
    return math.radians(d-90.0)


def _environment_source_lines(snapshot):
    loc=snapshot.get('location',{}); wmm=snapshot.get('wmm',{}); gm=snapshot.get('geomag',{}); aur=snapshot.get('aurora',{}); cr=snapshot.get('cosmic_ray',{}); sw=snapshot.get('swpc',{})
    lines=[]
    if loc.get('lat') is None: lines.append('LOCATION: NOT SET')
    else: lines.append(f"LOC {float(loc['lat']):+.4f}, {float(loc['lon']):+.4f}")
    f=_float_or_none(wmm.get('f')); d=_float_or_none(wmm.get('d')); i=_float_or_none(wmm.get('i'))
    lines.append('WMM2025 unavailable' if f is None else f"WMM exact {f/1000:.2f}uT D {d:+.1f}° I {i:+.1f}°")
    df=_float_or_none(gm.get('df')); dist=_float_or_none(gm.get('distance_km'))
    lines.append('GEOMAG unavailable' if not gm.get('available') else f"{gm.get('provider','OBS')} {gm.get('station','?')} {dist:.0f}km dF {df:+.1f}nT" if df is not None and dist is not None else f"{gm.get('provider','OBS')} {gm.get('station','?')}")
    ap=_float_or_none(aur.get('probability')); lines.append('OVATION unavailable' if ap is None else f"OVATION local grid {ap:.1f}%")
    c=_float_or_none(cr.get('counts_per_sec')); rel=_float_or_none(cr.get('relative_percent')); cd=_float_or_none(cr.get('distance_km'))
    lines.append('COSMIC RAYS unavailable' if c is None else f"{cr.get('station') or 'NM'} {c:.2f}/s d={rel:+.2f}% {cd:.0f}km" if rel is not None and cd is not None else f"{cr.get('station') or 'NM'} {c:.2f}/s")
    kp=_float_or_none(sw.get('kp')); speed=_float_or_none(sw.get('solar_wind_speed')); bz=_float_or_none(sw.get('bz_gsm'))
    lines.append('SWPC unavailable' if kp is None and speed is None else f"SWPC Kp {kp or 0:.2f} wind {speed or 0:.0f}km/s Bz {bz or 0:+.1f}nT")
    return lines


def _draw_environment_panel(layer,snapshot,title):
    draw=ImageDraw.Draw(layer); w,h=layer.size; pal=_env_palette(220); font=_font(max(8,min(11,w//48))); title_font=_font(max(10,min(14,w//38)))
    lines=_environment_source_lines(snapshot); pw=min(w-12,max(230,int(w*.72))); ph=25+len(lines)*14
    draw.rounded_rectangle([(6,6),(pw,6+ph)],radius=7,fill=pal['panel'],outline=pal['field2'],width=1); draw.text((13,11),title,font=title_font,fill=pal['field'])
    for n,line in enumerate(lines): draw.text((13,31+n*14),line,font=font,fill=(230,235,240,220) if viz_settings.get('overlay_tone')=='color' else pal['field'])


def fx_environment_accurate(img, people, frame_num, intensity):
    """FIELD PLATE: contours, measured vector, and sparse directional field."""
    snap = _environment_snapshot()
    arr = _science_plate_base(img, strength=0.52)
    base = Image.fromarray(arr).convert("RGBA")
    layer = Image.new("RGBA", base.size, (0,0,0,0))
    draw = ImageDraw.Draw(layer)
    w, h = base.size
    md, _ = _env_drives(snap)
    theta = _env_field_angle(snap)
    _fine_contours(layer, img, alpha=int(34 + 48*(intensity/100)))
    _field_streamlines(layer, theta, md, frame_num, intensity)

    gm = snap.get("geomag", {})
    dx = _float_or_none(gm.get("dx")) or 0.0
    dy = _float_or_none(gm.get("dy")) or 0.0
    cx, cy = int(w*0.82), int(h*0.21)
    rr = max(15, int(min(w,h)*0.075))
    draw.ellipse((cx-rr,cy-rr,cx+rr,cy+rr), outline=(220,230,225,100), width=1)
    draw.line((cx-rr-5,cy,cx+rr+5,cy), fill=(220,230,225,55), width=1)
    draw.line((cx,cy-rr-5,cx,cy+rr+5), fill=(220,230,225,55), width=1)
    if dx or dy:
        norm = max(1e-6, math.hypot(dx,dy))
        ll = rr*0.9
        ex, ey = cx+dx/norm*ll, cy+dy/norm*ll
        draw.line((cx,cy,ex,ey), fill=(243,220,174,210), width=2)

    _small_science_annotation(layer, snap, "FIELD PLATE / LIVE FIELD DATA", intensity)
    return Image.alpha_composite(base, layer).convert("RGB")


def _environment_field_warp(img,snap,frame_num,intensity,multiplier=1.0):
    arr=np.array(img.convert("RGB"))
    h,w=arr.shape[:2]
    xn,yn=_normalized_grids(h,w)
    theta=_env_field_angle(snap)
    md,_=_env_drives(snap)
    ss=np.clip(float(intensity)/100,0,1)
    amp=(1.0+12*md)*ss*multiplier
    phase=frame_num*(.013+.035*md)
    along=xn*math.cos(theta)+yn*math.sin(theta)
    across=-xn*math.sin(theta)+yn*math.cos(theta)
    wave=np.sin(along*(3.2+5.0*md)+phase)*np.exp(-0.22*across*across)
    curl=np.cos(across*(3.8+1.7*md)-phase*.65)
    dx=amp*(wave*math.cos(theta+math.pi/2)+.24*curl*math.cos(theta))
    dy=amp*(wave*math.sin(theta+math.pi/2)+.24*curl*math.sin(theta))
    return Image.fromarray(_warp_image(arr,dx,dy))


def fx_environment_field(img, people, frame_num, intensity):
    """FIELD WARP: measured-direction streamlines plus restrained refraction."""
    snap = _environment_snapshot()
    warped = _environment_field_warp(img,snap,frame_num,intensity,1.0)
    arr = _science_plate_base(warped, strength=0.36)
    base = Image.fromarray(arr).convert("RGBA")
    layer = Image.new("RGBA", base.size, (0,0,0,0))
    md,_ = _env_drives(snap)
    theta = _env_field_angle(snap)
    _field_streamlines(layer,theta,min(1.0,md*1.25),frame_num,intensity)
    _fine_contours(layer,warped,alpha=int(24+34*(intensity/100)))
    _small_science_annotation(layer,snap,"FIELD WARP / MAGNETIC STREAMLINES",intensity)
    return Image.alpha_composite(base,layer).convert("RGB")


def _draw_data_particles(layer,snap,frame_num,intensity,extreme=False):
    """Compatibility wrapper used by composite mode."""
    _draw_cloud_chamber(layer,snap,frame_num,intensity,dense=extreme)


def fx_environment_particles(img, people, frame_num, intensity):
    """CLOUD CHAMBER: persistent condensation tracks driven by live data."""
    snap = _environment_snapshot()
    ss = np.clip(float(intensity)/100.0,0.0,1.0)
    arr = _science_plate_base(img,strength=0.72).astype(np.float32)
    gray = cv2.cvtColor(arr.astype(np.uint8),cv2.COLOR_RGB2GRAY).astype(np.float32)
    chamber = np.dstack([
        gray*(0.43-0.08*ss),
        gray*(0.48-0.07*ss),
        gray*(0.46-0.07*ss)
    ]) + 4.0
    base = Image.fromarray(np.clip(chamber,0,255).astype(np.uint8)).convert("RGBA")
    layer = Image.new("RGBA",base.size,(0,0,0,0))
    _draw_cloud_chamber(layer,snap,frame_num,intensity,dense=False)
    _small_science_annotation(layer,snap,"CLOUD CHAMBER / DATA-DRIVEN SCHEMATIC",intensity)
    return Image.alpha_composite(base,layer).convert("RGB")


def fx_environment_abstract(img, people, frame_num, intensity):
    """RESONANCE PLATE: interference, contours, field lines, chamber traces."""
    snap = _environment_snapshot()
    md,pd = _env_drives(snap)
    theta = _env_field_angle(snap)
    ss = np.clip(float(intensity)/100.0,0.0,1.0)
    warped = _environment_field_warp(img,snap,frame_num,intensity,1.65)
    arr = _science_plate_base(warped,strength=0.62)
    h,w = arr.shape[:2]
    yy,xx = np.mgrid[0:h,0:w].astype(np.float32)
    xn=(xx/max(1,w-1))*2-1
    yn=(yy/max(1,h-1))*2-1
    a=xn*math.cos(theta)+yn*math.sin(theta)
    b=-xn*math.sin(theta)+yn*math.cos(theta)
    phase=frame_num*(0.010+0.018*md)
    waves=np.sin(a*(11+14*md)+phase)+np.cos(b*(13+17*pd)-phase*.8)
    ridge=np.exp(-np.abs(waves)*3.5)
    ridge=cv2.GaussianBlur(ridge.astype(np.float32),(0,0),0.7)
    toned=arr.astype(np.float32)
    if viz_settings.get("overlay_tone","color")=="bw":
        ink=np.dstack([ridge,ridge,ridge])*45*ss
    else:
        ink=np.dstack([ridge*26,ridge*43,ridge*39])*ss
    toned=np.clip(toned*0.88+ink,0,255).astype(np.uint8)
    base=Image.fromarray(toned).convert("RGBA")
    layer=Image.new("RGBA",base.size,(0,0,0,0))
    _fine_contours(layer,Image.fromarray(arr),alpha=int(34+56*ss))
    _field_streamlines(layer,theta+0.16,md,frame_num,intensity*0.72)
    _draw_cloud_chamber(layer,snap,frame_num,intensity*0.58,dense=True)
    _small_science_annotation(layer,snap,"RESONANCE PLATE / INTERFERENCE",intensity)
    return Image.alpha_composite(base,layer).convert("RGB")

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  HUD
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def draw_hud(img, mode_label, people_count, frame_num):
    """Restrained scientific annotation strip instead of a large dashboard."""
    snap=_environment_snapshot()
    sw=snap.get("swpc",{})
    gm=snap.get("geomag",{})
    cosmic=snap.get("cosmic_ray",{})
    with _detection_lock:
        total=int(detection_data.get("total_seen",0))

    pieces=[mode_label]
    kp=_float_or_none(sw.get("kp"))
    bz=_float_or_none(sw.get("bz_gsm"))
    df=_float_or_none(gm.get("df"))
    cr=_float_or_none(cosmic.get("relative_percent"))
    if kp is not None: pieces.append(f"Kp {kp:.1f}")
    if bz is not None: pieces.append(f"Bz {bz:+.1f}")
    if df is not None: pieces.append(f"dF {df:+.0f}nT")
    if cr is not None: pieces.append(f"{cosmic.get('station') or 'NM'} {cr:+.2f}%")
    if people_count: pieces.append(f"subjects {people_count}/{total}")

    text="  ·  ".join(pieces)
    ov=Image.new("RGBA",img.size,(0,0,0,0))
    draw=ImageDraw.Draw(ov)
    font=_font(max(8,min(10,img.size[0]//70)))
    bbox=draw.textbbox((0,0),text,font=font)
    bw=min(img.size[0]-12,(bbox[2]-bbox[0])+16)
    draw.rectangle((6,6,6+bw,24),fill=(3,6,7,105))
    draw.text((12,10),text,font=font,fill=(220,228,224,190))
    return Image.alpha_composite(img.convert("RGBA"),ov).convert("RGB")
def make_bw_sharpened(img, strength=1.65, blur_sigma=1.1):
    """Convert the camera feed to crisp black-and-white before color overlays."""
    arr = np.array(img.convert('RGB'))
    gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (0, 0), blur_sigma)
    sharp = cv2.addWeighted(gray, strength, blur, -(strength - 1.0), 0)
    sharp = np.clip(sharp, 0, 255).astype(np.uint8)
    rgb = cv2.cvtColor(sharp, cv2.COLOR_GRAY2RGB)
    return Image.fromarray(rgb)


def prepare_camera_frame(frame):
    """Rotate camera frame; image color and overlay color are independent."""
    rgb=np.ascontiguousarray(frame[:,:,:3]); rotated=cv2.rotate(rgb,cv2.ROTATE_90_COUNTERCLOCKWISE)
    if viz_settings.get('image_tone','bw')=='color':
        blur=cv2.GaussianBlur(rotated,(0,0),1.0); sharp=cv2.addWeighted(rotated,1.35,blur,-.35,0); return Image.fromarray(np.clip(sharp,0,255).astype(np.uint8))
    gray=cv2.cvtColor(rotated,cv2.COLOR_RGB2GRAY); blur=cv2.GaussianBlur(gray,(0,0),1.1); sharp=cv2.addWeighted(gray,1.65,blur,-.65,0); return Image.fromarray(cv2.cvtColor(np.clip(sharp,0,255).astype(np.uint8),cv2.COLOR_GRAY2RGB))

def render_processed_frame(frame, include_hud=True, learn_background=False):
    """Render a lores preview or native-resolution main-stream still."""
    raw_h, raw_w = frame.shape[:2]
    img = prepare_camera_frame(frame)

    # YOLO runs only on the 1280×720 lores stream.  Scale its tracked boxes to
    # the stream currently being rendered, then rotate them with the image.
    raw_people, raw_trails, total_seen, detect_status, used_stale = _smooth_people_for_render()
    scaled_people = scale_boxes(raw_people, PREVIEW_SIZE, (raw_w, raw_h))
    scaled_trails = [scale_boxes(t, PREVIEW_SIZE, (raw_w, raw_h)) for t in raw_trails]
    people = rotate_boxes_90ccw(scaled_people, orig_w=raw_w)
    trails = [rotate_boxes_90ccw(t, orig_w=raw_w) for t in scaled_trails]

    mode      = viz_settings.get('mode', 'panopticon')
    intensity = int(np.clip(viz_settings.get('intensity', 70), 0, 100))

    # Scan validation runs on every preview frame. Adaptive learning is capped
    # to reduce CPU use while still warming in the background before Erase mode.
    if learn_background:
        with _scene_map_lock:
            scan_active = bool(_scene_scan_state.get('active'))
        if scan_active and (_frame_count % SCENE_SCAN_EVERY) == 0:
            update_scene_scan(img, people)
        if (_frame_count % BACKGROUND_LEARN_EVERY) == 0:
            update_background_model(img, people)

    if   mode == 'env_accurate':  out = fx_environment_accurate(img, people, _frame_count, intensity); label='FIELD PLATE'
    elif mode == 'env_field':     out = fx_environment_field(img, people, _frame_count, intensity); label='FIELD WARP'
    elif mode == 'env_particles': out = fx_environment_particles(img, people, _frame_count, intensity); label='CLOUD CHAMBER'
    elif mode == 'env_abstract':  out = fx_environment_abstract(img, people, _frame_count, intensity); label='RESONANCE PLATE'
    elif mode == 'panopticon':    out = fx_panopticon(img, people, _frame_count, intensity); label='PANOPTICON'
    elif mode == 'ghost':         out = fx_ghost(img, people, trails, _frame_count, intensity); label='AFTERIMAGE'
    elif mode == 'dissolution': out = fx_dissolution(img, people, _frame_count, intensity); label='EMULSION'
    elif mode == 'shield':      out = fx_shield(img, people, _frame_count, intensity); label='FARADAY VEIL'
    elif mode == 'erased':      out = fx_erased(img, people, _frame_count, intensity, source='adaptive'); label='ERASED'
    elif mode == 'mapped':      out = fx_erased(img, people, _frame_count, intensity, source='mapped'); label='MAPPED ERASE'
    elif mode == 'all':
        out = fx_ghost(img, people, trails, _frame_count, intensity * 0.5)
        out = fx_dissolution(out, people, _frame_count, intensity * 0.5)
        out = fx_shield(out, people, _frame_count, intensity * 0.6)
        out = fx_panopticon(out, people, _frame_count, intensity * 0.4)
        label = 'COMPOSITE'
    elif mode == 'hyperspace':  out = fx_hyperspace(img, people, _frame_count, intensity); label='4D HYPERSPACE'
    elif mode == 'raw':         out = fx_raw(img); label='RAW CAMERA'
    else:                       out = img; label='CAMERA'

    out = apply_tap_removals(out)
    if include_hud:
        out = draw_hud(out, label, len(people), _frame_count)
    if out.mode != 'RGB':
        out = out.convert('RGB')
    return out


def capture_current_frame(prefix='antisurv', source='web'):
    """Capture one processed 2K/QHD still from the main camera stream."""
    try:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        fname = os.path.join(CAPTURE_DIR, f'{prefix}_{ts}.jpg')
        tmp_name = fname + '.tmp'
        frame = _capture_stream_array('main')
        img = render_processed_frame(frame, include_hud=True, learn_background=False)
        img.save(
            tmp_name,
            'JPEG',
            quality=STILL_JPEG_QUALITY,
            subsampling=0,
            optimize=False,
        )
        os.replace(tmp_name, fname)
        _notify_capture(True, filename=fname, source=source)
        return {
            'success': True,
            'filename': fname,
            'display_name': os.path.basename(fname),
            'source': source,
            'width': img.width,
            'height': img.height,
            'quality': STILL_JPEG_QUALITY,
        }
    except Exception as e:
        try:
            if 'tmp_name' in locals() and os.path.exists(tmp_name):
                os.remove(tmp_name)
        except Exception:
            pass
        _notify_capture(False, source=source, error=str(e))
        return {'success': False, 'error': str(e), 'source': source}


def _local_cycle_mode(step):
    """Cycle visual mode from the joystick/touchscreen."""
    with _local_control_lock:
        current = viz_settings.get('mode', LOCAL_MODE_ORDER[0])
        try:
            idx = LOCAL_MODE_ORDER.index(current)
        except ValueError:
            idx = 0
        mode = LOCAL_MODE_ORDER[(idx + int(step)) % len(LOCAL_MODE_ORDER)]
        viz_settings['mode'] = mode
    print(f"Local control: mode -> {mode}")
    return mode


def _local_adjust_intensity(delta):
    """Adjust intensity; log only when the value actually changes."""
    with _local_control_lock:
        old_value = int(viz_settings.get('intensity', 78))
        value = int(np.clip(old_value + int(delta), 0, 100))
        viz_settings['intensity'] = value
    if value != old_value:
        print(f"Local control: intensity -> {value}%")
    return value



def _local_toggle_image_tone():
    with _local_control_lock:
        cur=viz_settings.get('image_tone','bw'); viz_settings['image_tone']='color' if cur=='bw' else 'bw'; value=viz_settings['image_tone']
    print(f"Local control: image tone -> {value}"); return value

def _local_toggle_overlay_tone():
    with _local_control_lock:
        cur=viz_settings.get('overlay_tone','color'); viz_settings['overlay_tone']='bw' if cur=='color' else 'color'; value=viz_settings['overlay_tone']
    print(f"Local control: overlay tone -> {value}"); return value

def _local_capture(source='joystick'):
    """Run a still capture in a worker so UI input never blocks."""
    def _work():
        result = capture_current_frame(prefix='antisurv_local', source=source)
        if result.get('success'):
            print(f"📸 Local capture ({source}): {result.get('filename')}")
        else:
            print(f"Local capture error ({source}): {result.get('error')}")
    threading.Thread(target=_work, daemon=True, name=f'capture-{source}').start()


# ─── Independent capture / artistic effects / MJPEG streams ─────────────────
# Capture is never blocked by expensive artistic renders or SPI LCD transfers.
# Latest-frame buffers contain ONE frame, not a growing queue. A slow browser
# or a slow SPI display skips frames instead of building up latency.
_stream_condition = threading.Condition()
_stream_jpeg = None
_stream_sequence = 0
_stream_started = False
_stream_start_lock = threading.Lock()
_stream_clients = 0
_stream_stats = {'fps': 0.0, 'frame_ms': 0.0, 'quality': PREVIEW_JPEG_QUALITY,
                 'mode': PREVIEW_MODE, 'last_frame_monotonic': 0.0}
_preview_mode = PREVIEW_MODE
_local_preview_lock = threading.Lock()
_local_preview_image = None
_local_preview_sequence = 0
_live_preview_capture_announced = False
_live_preview_lcd_queue_announced = False
_live_tft_write_announced = False
_effect_lcd_seen = False
_lcd_last_raw_publish = 0.0


def _publish_web_frame(image_rgb, started, kind):
    """Encode once for all connected browsers and publish only the newest JPEG."""
    global _stream_jpeg, _stream_sequence
    # The raw path receives ndarray RGB; the effect path receives PIL RGB.
    if LOCAL_ONLY:
        return
    if isinstance(image_rgb, Image.Image):
        rgb = np.asarray(image_rgb.convert('RGB'))
    else:
        rgb = image_rgb
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode('.jpg', bgr, [
        cv2.IMWRITE_JPEG_QUALITY, PREVIEW_JPEG_QUALITY,
        cv2.IMWRITE_JPEG_OPTIMIZE, 0,
    ])
    if not ok:
        raise RuntimeError('preview JPEG encoding failed')
    now = time.monotonic()
    with _stream_condition:
        if _preview_mode != kind:
            return
        prev = _stream_stats.get('last_frame_monotonic', 0.0)
        fps = (1.0 / max(1e-4, now-prev)) if prev and _stream_stats['mode'] == kind else 0.0
        _stream_stats.update({
            'fps': round(fps, 1), 'frame_ms': round((now-started)*1000.0, 1),
            'last_frame_monotonic': now, 'mode': kind,
        })
        _stream_jpeg = encoded.tobytes()
        _stream_sequence += 1
        _stream_condition.notify_all()


def stream_producer_loop():
    """Capture lores continuously; fast JPEGs never wait for FX or SPI."""
    global _frame_count, _latest_frame, _latest_frame_id
    global _local_preview_image, _local_preview_sequence
    global _live_preview_capture_announced, _live_preview_lcd_queue_announced
    global _lcd_last_raw_publish

    period = 1.0 / max(1.0, PREVIEW_FPS)
    next_deadline = time.monotonic()
    while not _shutdown_event.is_set():
        started = time.monotonic()
        try:
            frame = _capture_stream_array('lores')
            with _frame_lock:
                _latest_frame = frame     # immutable after publication
                _latest_frame_id += 1
            _frame_count += 1
            if not _live_preview_capture_announced:
                print(f"✓ Camera frame producer: {frame.shape[1]}x{frame.shape[0]} "
                      f"@ target {PREVIEW_FPS:g} fps, web defaults to FAST")
                _live_preview_capture_announced = True

            # Publish at most LCD_FPS *latest* raw frames to TFT. The producer
            # never waits for slow SPI transactions; local_lcd_loop handles those.
            # This path does not run the artwork renderer on every TFT frame.
            if HAS_LCD and LCD_SOURCE == 'raw' and (started - _lcd_last_raw_publish >= 1.0 / max(1., LCD_FPS)):
                lcd_rgb = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                raw_img = Image.fromarray(lcd_rgb)
                with _local_preview_lock:
                    _local_preview_image = raw_img
                    _local_preview_sequence += 1
                _lcd_last_raw_publish = started
                if not _live_preview_lcd_queue_announced:
                    print('✓ TFT raw preview isolated from FX renderer', flush=True)
                    _live_preview_lcd_queue_announced = True

            if not LOCAL_ONLY and _preview_mode == 'fast':
                # Rotating the RGB ndarray is far cheaper than running all
                # artistic effects + PIL layers + YOLO before JPEG encoding.
                raw_rgb = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                _publish_web_frame(raw_rgb, started, 'fast')
        except Exception as e:
            print(f"Camera/web FAST frame error: {type(e).__name__}: {e}", flush=True)
            _shutdown_event.wait(0.15)

        next_deadline += period
        now = time.monotonic()
        if next_deadline < now - period:
            next_deadline = now
        if next_deadline > now:
            _shutdown_event.wait(next_deadline - now)


def effect_preview_loop():
    """Render effects separately at a capped rate for LCD and optional FX web.

    In FAST + no LCD mode this worker sleeps until FX is requested or an
    explicit scene-mapping job needs processed preview frames.
    """
    global _local_preview_image, _local_preview_sequence, _effect_lcd_seen
    last_frame_id = -1
    period = 1.0 / max(1.0, min(PREVIEW_EFFECT_FPS, LCD_FX_FPS) if LOCAL_ONLY else PREVIEW_EFFECT_FPS)
    while not _shutdown_event.is_set():
        started = time.monotonic()
        with _scene_map_lock:
            scan_active = bool(_scene_scan_state.get('active'))
        if not ((HAS_LCD and LCD_SOURCE == 'fx') or (not LOCAL_ONLY and _preview_mode == 'fx') or scan_active):
            _shutdown_event.wait(0.15)
            continue
        with _frame_lock:
            seq = _latest_frame_id
            frame = (_latest_frame.copy()
                     if _latest_frame is not None and seq != last_frame_id
                     else None)
        if frame is None:
            _shutdown_event.wait(0.02)
            continue
        last_frame_id = seq
        try:
            processed = render_processed_frame(frame, include_hud=True,
                                               learn_background=True)
            if HAS_LCD and LCD_SOURCE == 'fx':
                with _local_preview_lock:
                    _local_preview_image = processed
                    _local_preview_sequence += 1
                    _effect_lcd_seen = True
            if not LOCAL_ONLY and _preview_mode == 'fx':
                _publish_web_frame(processed, started, 'fx')
        except Exception as e:
            print(f"Artistic FX frame error: {type(e).__name__}: {e}", flush=True)
            _shutdown_event.wait(0.10)
        elapsed = time.monotonic() - started
        if elapsed < period:
            _shutdown_event.wait(period - elapsed)


def ensure_stream_producer():
    global _stream_started
    with _stream_start_lock:
        if not _stream_started:
            threading.Thread(target=stream_producer_loop, daemon=True,
                             name='camera-fast-producer').start()
            threading.Thread(target=effect_preview_loop, daemon=True,
                             name='camera-art-fx').start()
            _stream_started = True


def generate_frames():
    global _stream_clients
    ensure_stream_producer()
    last_sequence = -1
    with _stream_condition:
        _stream_clients += 1
    try:
        while not _shutdown_event.is_set():
            with _stream_condition:
                new_frame = _stream_condition.wait_for(
                    lambda: _stream_jpeg is not None and _stream_sequence != last_sequence,
                    timeout=2.0,
                )
                if not new_frame:
                    continue
                jpeg = _stream_jpeg
                last_sequence = _stream_sequence
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                   + f'Content-Length: {len(jpeg)}\r\n'.encode('ascii')
                   + b'Cache-Control: no-store\r\n\r\n' + jpeg + b'\r\n')
    finally:
        with _stream_condition:
            _stream_clients = max(0, _stream_clients - 1)

# ─── Local TFT rendering and physical controls ────────────────────────────────
# The physical camera UI is deliberately simple:
#   LEFT/RIGHT : previous/next visualization mode
#   UP/DOWN    : effect intensity
#   CLICK      : open/close a small settings overlay
#   SHUTTER    : dedicated GPIO27 button only
_local_menu_open = False
_local_menu_index = 0
_LOCAL_MENU_ITEMS = ("image_tone", "overlay_tone", "exit")
_local_ui_revision = 0
_local_menu_last_input = 0.0


def _touch_local_ui():
    global _local_ui_revision, _local_menu_last_input
    _local_ui_revision += 1
    _local_menu_last_input = time.monotonic()


def _lcd_text(draw, xy, text, fill=(240, 242, 245), size=11):
    draw.text(xy, str(text), font=_font(size), fill=fill)


def _fit_for_lcd(img):
    """Landscape 320x240: left 2/3 camera, right 1/3 controls.

    The settings menu replaces sidebar rows; it never obscures the preview.
    All dimensions are obtained from the initialized panel driver.
    """
    w, h = _lcd_width, _lcd_height
    if w <= h:
        # Do not silently render an unusably narrow sidebar in portrait.
        # Select LCD_ROTATION=90 or 270 (driver may vary by mounting).
        if not getattr(_fit_for_lcd, "_warned", False):
            print(f"TFT layout warning: panel reports {w}x{h}, expected landscape. "
                  "Try LCD_ROTATION=90 or LCD_ROTATION=270.")
            _fit_for_lcd._warned = True

    sidebar_w = max(72, w // 3)
    preview_w = w - sidebar_w
    canvas = Image.new("RGB", (w, h), (7, 9, 13))
    if img is not None:
        preview = ImageOps.fit(
            img.convert("RGB"), (preview_w, h),
            method=Image.Resampling.BILINEAR, centering=(0.5, 0.5)
        )
        canvas.paste(preview, (0, 0))

    draw = ImageDraw.Draw(canvas)
    x = preview_w
    draw.rectangle((x, 0, w - 1, h - 1), fill=(12, 16, 22))
    draw.line((x, 0, x, h), fill=(94, 103, 115), width=1)
    pad = 7
    sx = x + pad
    sw = w - sx - pad

    if _local_menu_open:
        _lcd_text(draw, (sx, 11), "SETTINGS", size=12)
        draw.line((sx, 32, w - pad, 32), fill=(75, 83, 96))
        options = [
            ("IMAGE", str(viz_settings.get("image_tone", "bw")).upper()),
            ("OVERLAY", str(viz_settings.get("overlay_tone", "color")).upper()),
            ("EXIT", ""),
        ]
        for i, (key, value) in enumerate(options):
            yy = 42 + i * 55
            active = i == (_local_menu_index % len(options))
            if active:
                draw.rounded_rectangle((x + 3, yy - 3, w - 4, yy + 43),
                                       radius=5, fill=(40, 50, 62))
            _lcd_text(draw, (sx, yy), (">" if active else " ") + key,
                      size=11)
            if value:
                _lcd_text(draw, (sx + 9, yy + 20), value,
                          fill=(177, 212, 226), size=11)
        _lcd_text(draw, (sx, h - 18), "CLICK: SELECT", size=9,
                  fill=(160, 170, 184))
    else:
        mode = viz_settings.get("mode", "env_accurate")
        mode_label = LOCAL_MODE_LABELS.get(mode, str(mode).replace("_", " ").upper())
        _lcd_text(draw, (sx, 9), "MODE", fill=(158, 176, 191), size=10)
        # Wrap mode names rather than truncating important distinctions.
        from textwrap import wrap
        lines = wrap(str(mode_label).upper(), width=max(7, sw // 7),
                     break_long_words=True, break_on_hyphens=False)[:3]
        for j, line in enumerate(lines):
            _lcd_text(draw, (sx, 27 + j * 15), line, size=12)
        divider_y = max(80, 30 + len(lines) * 17)
        draw.line((sx, divider_y, w - pad, divider_y), fill=(75, 83, 96))

        intensity = int(viz_settings.get("intensity", 78))
        _lcd_text(draw, (sx, divider_y + 9), "INTENSITY", size=9,
                  fill=(158, 176, 191))
        _lcd_text(draw, (sx, divider_y + 24), f"{intensity}%", size=17)
        bar_top = divider_y + 51
        bar_end = min(w - pad, sx + sw)
        draw.rounded_rectangle((sx, bar_top, bar_end, bar_top + 7),
                               radius=3, fill=(47, 55, 65))
        if intensity > 0:
            draw.rounded_rectangle((sx, bar_top,
                                    sx + int(sw * max(0, min(100, intensity)) / 100),
                                    bar_top + 7), radius=3, fill=(192, 211, 228))
        if bar_top + 42 < h - 21:
            image_tone = str(viz_settings.get("image_tone", "bw")).upper()
            overlay_tone = str(viz_settings.get("overlay_tone", "color")).upper()
            _lcd_text(draw, (sx, bar_top + 20), f"IMG {image_tone}", size=9)
            _lcd_text(draw, (sx, bar_top + 35), f"FX  {overlay_tone}", size=9)
        draw.line((sx, h - 28, w - pad, h - 28), fill=(75, 83, 96))
        _lcd_text(draw, (sx, h - 20), "CLICK: MENU", size=9,
                  fill=(160, 170, 184))
    return canvas


# Portfolio exports use the *actual* composed LCD frame, including the real
# camera/effect preview, current menu, or photo review. The renderer never waits
# for PNG compression; a bounded queue drops exports if the disk is too slow.
UI_EXPORT_DIR = Path(os.environ.get("UI_EXPORT_DIR", str(Path.home() / "resonance-ui-exports")))
UI_EXPORT_WIDTH = max(320, min(4096, int(os.environ.get("UI_EXPORT_WIDTH", "1920"))))
UI_EXPORT_SCALE_MODE = os.environ.get("UI_EXPORT_SCALE_MODE", "lanczos").lower()
_ui_export_request = threading.Event()
_ui_export_jobs = queue.Queue(maxsize=2)
_ui_export_num = 0


def _request_ui_export(*_args):
    # Safe to invoke from a signal handler: the LCD loop prepares the snapshot.
    _ui_export_request.set()


def _queue_ui_export(frame):
    if not _ui_export_request.is_set() or frame is None:
        return
    _ui_export_request.clear()
    try:
        _ui_export_jobs.put_nowait(frame.copy())
    except queue.Full:
        print('UI export skipped: previous exports still being saved', flush=True)


def _ui_export_worker():
    global _ui_export_num
    while not _shutdown_event.is_set():
        try:
            frame = _ui_export_jobs.get(timeout=0.4)
        except queue.Empty:
            continue
        try:
            UI_EXPORT_DIR.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            _ui_export_num += 1
            base = f'resonance_ui_{stamp}_{_ui_export_num:03d}'
            native = UI_EXPORT_DIR / (base + '_native.png')
            hd = UI_EXPORT_DIR / (base + '_HD.png')
            frame.save(native, 'PNG')
            output_h = round(UI_EXPORT_WIDTH * frame.height / frame.width)
            resample = (Image.Resampling.NEAREST if UI_EXPORT_SCALE_MODE == 'nearest'
                        else Image.Resampling.LANCZOS)
            frame.resize((UI_EXPORT_WIDTH, output_h), resample=resample).save(hd, 'PNG')
            print(f'UI EXPORT: {hd} (source: {native})', flush=True)
        except Exception as exc:
            print(f'UI EXPORT ERROR: {exc}', flush=True)
        finally:
            _ui_export_jobs.task_done()


def local_lcd_loop():
    """Single SPI writer: photo status/review has priority over live frames."""
    global _live_tft_write_announced
    if not HAS_LCD or _lcd_bridge is None:
        return
    last_seq, last_ui_revision, last_photo_revision = -1, -1, -1
    period = 1.0 / max(1.0, LCD_FPS)
    consecutive_errors = 0
    while not _shutdown_event.is_set():
        started = time.monotonic()
        try:
            with _photo_lock:
                photo_rev, photo_state = _photo_revision, _photo_status
            with _local_preview_lock:
                seq = _local_preview_sequence
                ui_rev = _local_ui_revision
                changed = seq != last_seq or ui_rev != last_ui_revision
                frame = _local_preview_image.copy() if _local_preview_image is not None and changed else None
            image = None
            if photo_state != 'live':
                # Static review has no need for ongoing SPI writes.
                if photo_rev != last_photo_revision:
                    image = _review_canvas()
                else:
                    _review_canvas()  # expires timed review even if no frames change
            elif frame is not None or photo_rev != last_photo_revision:
                image = _fit_for_lcd(frame if frame is not None else _local_preview_image)
            if _ui_export_request.is_set():
                if image is None:
                    if photo_state != 'live':
                        image = _review_canvas()
                    else:
                        with _local_preview_lock:
                            source = _local_preview_image.copy() if _local_preview_image is not None else None
                        image = _fit_for_lcd(source)
                _queue_ui_export(image)
            if image is not None:
                ok = _lcd_write_image(image)
                if ok:
                    if not _live_tft_write_announced:
                        print('✓ First live TFT frame written', flush=True)
                        _live_tft_write_announced = True
                    last_seq, last_ui_revision, last_photo_revision = seq, ui_rev, photo_rev
                    consecutive_errors = 0
                else:
                    consecutive_errors += 1
            elif photo_state == 'live' and frame is None and _local_preview_image is None and consecutive_errors == 0:
                # No frame available yet; do not write a static placeholder repeatedly.
                pass
        except Exception as exc:
            consecutive_errors += 1
            if consecutive_errors <= 3 or consecutive_errors % 20 == 0:
                print(f'Local TFT error #{consecutive_errors}: {type(exc).__name__}: {exc}', flush=True)
            _shutdown_event.wait(0.25)
        elapsed = time.monotonic() - started
        if elapsed < period:
            _shutdown_event.wait(period-elapsed)


def _menu_change(_delta=1):
    global _local_menu_open
    item = _LOCAL_MENU_ITEMS[_local_menu_index % len(_LOCAL_MENU_ITEMS)]
    if item == "image_tone":
        _local_toggle_image_tone()
    elif item == "overlay_tone":
        _local_toggle_overlay_tone()
    elif item == "exit":
        _local_menu_open = False
    _touch_local_ui()


def _calibrate_joystick(samples=80, settle=0.45):
    """Calibrate centered joystick and reject rail-pinned/invalid analog inputs.

    A 3.3 V powered joystick should normally center well away from either rail.
    Returning valid=False prevents accidental mode/intensity changes when VRX/VRY
    are miswired, floating, or tied to VCC/GND.
    """
    print("Joystick calibration: leave stick centered...")
    time.sleep(settle)
    xs, ys = [], []
    for _ in range(samples):
        try:
            xs.append(float(_joy_x.voltage))
            ys.append(float(_joy_y.voltage))
        except Exception:
            pass
        time.sleep(0.012)

    if len(xs) < 20 or len(ys) < 20:
        print("✗ Joystick calibration failed: insufficient ADC samples.")
        return 1.65, 1.65, JOYSTICK_DEADZONE_V, JOYSTICK_DEADZONE_V, False

    cx = float(statistics.median(xs))
    cy = float(statistics.median(ys))
    mad_x = float(statistics.median(abs(v - cx) for v in xs))
    mad_y = float(statistics.median(abs(v - cy) for v in ys))

    dz_x = float(np.clip(max(JOYSTICK_DEADZONE_V, 6.0 * mad_x + 0.08), 0.25, 0.75))
    dz_y = float(np.clip(max(JOYSTICK_DEADZONE_V, 6.0 * mad_y + 0.08), 0.25, 0.75))

    # Keep plenty of headroom to both rails.  3.29 V on a 3.3 V system is
    # effectively rail-pinned and cannot represent a healthy centered stick.
    valid_x = 0.35 <= cx <= 2.95
    valid_y = 0.35 <= cy <= 2.95
    valid = bool(valid_x and valid_y)

    if valid:
        print(
            f"✓ Joystick calibrated: center X={cx:.2f}V Y={cy:.2f}V "
            f"deadzone X=±{dz_x:.2f}V Y=±{dz_y:.2f}V"
        )
    else:
        print(
            f"✗ Joystick analog inputs invalid for centered stick: "
            f"X={cx:.2f}V Y={cy:.2f}V"
        )
        print(
            "  Expected center is roughly mid-supply (~1.65V on a 3.3V joystick). "
            "Joystick movement controls are DISABLED until wiring is corrected."
        )
        print(
            "  Verify: joystick VCC→3.3V, GND→GND, VRX→ADS1115 A0, "
            "VRY→ADS1115 A1. Do NOT power the joystick from 5V while ADS1115 VDD is 3.3V."
        )

    return cx, cy, dz_x, dz_y, valid


def joystick_loop():
    global _local_menu_open, _local_menu_index
    if not HAS_JOYSTICK:
        return

    center_x, center_y, dead_x, dead_y, analog_valid = _calibrate_joystick()
    if not analog_valid:
        # Preserve the push switch if desired, but do not interpret rail-pinned
        # analog channels as directional commands.
        print("Joystick directional controls disabled; run with --joystick-test after rewiring.")
        while not _shutdown_event.is_set():
            _shutdown_event.wait(0.5)
        return
    last_h = 0
    last_v = 0
    last_h_time = 0.0
    last_v_time = 0.0
    last_sw = GPIO.HIGH if HAS_GPIO else 1

    while not _shutdown_event.is_set():
        try:
            xv = float(_joy_x.voltage)
            yv = float(_joy_y.voltage)
            now = time.monotonic()

            # Auto-close the settings overlay after inactivity.
            if (
                _local_menu_open
                and _local_menu_last_input > 0
                and now - _local_menu_last_input > LCD_MENU_TIMEOUT_SEC
            ):
                _local_menu_open = False
                _touch_local_ui()
                print("Local menu timeout -> closed")

            dx = xv - center_x
            dy = yv - center_y
            h = -1 if dx < -dead_x else (1 if dx > dead_x else 0)
            v = 1 if dy < -dead_y else (-1 if dy > dead_y else 0)

            if JOYSTICK_INVERT_X:
                h = -h
            if JOYSTICK_INVERT_Y:
                v = -v

            with _photo_lock:
                reviewing = _photo_status in ('review', 'saving', 'error')
            if reviewing:
                if _photo_status == 'review' and h and (h != last_h or now - last_h_time > 0.55):
                    _review_photo_step(h)
                    last_h_time = now
                if HAS_GPIO:
                    sw = GPIO.input(JOYSTICK_SW_PIN)
                    if last_sw == GPIO.HIGH and sw == GPIO.LOW:
                        if _photo_status != 'saving':
                            _close_photo_review()
                    last_sw = sw
                last_h, last_v = h, v
                _shutdown_event.wait(JOYSTICK_DEBOUNCE)
                continue

            if _local_menu_open:
                if v and (v != last_v or now - last_v_time > 0.50):
                    _local_menu_index = (_local_menu_index - v) % len(_LOCAL_MENU_ITEMS)
                    _touch_local_ui()
                    print(f"Local menu -> {_LOCAL_MENU_ITEMS[_local_menu_index]}")
                    last_v_time = now

                if h and (h != last_h or now - last_h_time > 0.55):
                    _menu_change(h)
                    last_h_time = now
            else:
                if h and (h != last_h or now - last_h_time > 0.60):
                    _local_cycle_mode(h)
                    _touch_local_ui()
                    last_h_time = now

                if v and (v != last_v or now - last_v_time > 0.45):
                    _local_adjust_intensity(5 * v)
                    _touch_local_ui()
                    last_v_time = now

            if HAS_GPIO:
                sw = GPIO.input(JOYSTICK_SW_PIN)
                if last_sw == GPIO.HIGH and sw == GPIO.LOW:
                    time.sleep(0.035)
                    if GPIO.input(JOYSTICK_SW_PIN) == GPIO.LOW:
                        if _local_menu_open:
                            item = _LOCAL_MENU_ITEMS[_local_menu_index % len(_LOCAL_MENU_ITEMS)]
                            if item == "exit":
                                _local_menu_open = False
                                print("Local menu closed")
                            else:
                                _menu_change(1)
                        else:
                            _local_menu_open = True
                            _local_menu_index = 0
                            print("Local menu opened")
                        _touch_local_ui()
                last_sw = sw

            last_h = h
            last_v = v

        except Exception as exc:
            print(f"Joystick read error: {type(exc).__name__}: {exc}")
            _shutdown_event.wait(0.4)

        _shutdown_event.wait(JOYSTICK_DEBOUNCE)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  HTML
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

HTML = '''
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Anti-Surveillance Camera — Media Lab Build</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500;600&display=swap');
  :root{
    --bg:#05060a;--panel:#0d1018;--panel2:#151925;--glass:rgba(13,16,24,.78);--border:#2a3042;
    --text:#e5e8ef;--muted:#7b8296;--dim:#4e566a;--green:#00ff7b;--cyan:#00e7ff;--red:#ff4052;
    --amber:#ffc247;--violet:#a78bfa;--violet2:#6d5dfc;--shadow:0 20px 60px rgba(0,0,0,.42);--radius:16px;
  }
  *{box-sizing:border-box} html,body{margin:0;min-height:100%;background:radial-gradient(circle at 20% 0%,#121827 0,#05060a 36%,#030409 100%);color:var(--text);font-family:Inter,system-ui,sans-serif;}
  body::before{content:"";position:fixed;inset:0;pointer-events:none;background:linear-gradient(rgba(255,255,255,.025) 1px,transparent 1px),linear-gradient(90deg,rgba(255,255,255,.018) 1px,transparent 1px);background-size:32px 32px;mask-image:linear-gradient(to bottom,rgba(0,0,0,.8),transparent 88%)}
  .topbar{height:58px;display:flex;align-items:center;justify-content:space-between;padding:0 18px;border-bottom:1px solid var(--border);background:rgba(7,9,14,.82);backdrop-filter:blur(18px);position:sticky;top:0;z-index:20}
  .brand{display:flex;align-items:center;gap:12px}.mark{width:32px;height:32px;border:1px solid rgba(0,255,123,.55);border-radius:50%;display:grid;place-items:center;color:var(--green);box-shadow:0 0 30px rgba(0,255,123,.16)}
  .brand h1{font-size:15px;line-height:1;margin:0;font-weight:700;letter-spacing:.01em}.brand small{display:block;color:var(--muted);font-size:10px;font-weight:500;margin-top:4px;letter-spacing:.09em;text-transform:uppercase}.live{font-family:'JetBrains Mono';font-size:11px;color:var(--green);letter-spacing:.11em}.live::before{content:"";display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--green);margin-right:8px;box-shadow:0 0 16px var(--green)}
  .layout{height:calc(100vh - 58px);display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:0}.stage{position:relative;background:#000;overflow:hidden;display:grid;place-items:center}.stage img{width:100%;height:100%;object-fit:contain;display:block}.stage.tap-mode{cursor:crosshair}.tap-overlay{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}.stage.tap-mode .tap-overlay{pointer-events:auto;touch-action:none}
  .stageHud{position:absolute;left:16px;top:16px;display:flex;gap:8px;flex-wrap:wrap;z-index:8}.chip{font-family:'JetBrains Mono';font-size:10px;letter-spacing:.06em;text-transform:uppercase;padding:7px 10px;border-radius:999px;background:rgba(0,0,0,.56);border:1px solid rgba(255,255,255,.13);backdrop-filter:blur(10px);color:#d7dceb}.chip.green{border-color:rgba(0,255,123,.4);color:var(--green)}.chip.violet{border-color:rgba(167,139,250,.5);color:var(--violet)}
  .modes{position:absolute;left:16px;bottom:16px;right:16px;display:flex;gap:7px;overflow-x:auto;padding:8px;border:1px solid rgba(255,255,255,.1);border-radius:18px;background:rgba(7,9,14,.68);backdrop-filter:blur(18px);box-shadow:var(--shadow);z-index:9}.modes::-webkit-scrollbar{display:none}.mode{font-family:'JetBrains Mono';font-size:10px;letter-spacing:.07em;text-transform:uppercase;white-space:nowrap;border:1px solid var(--border);border-radius:12px;background:rgba(21,25,37,.9);color:var(--muted);padding:10px 12px;cursor:pointer;transition:.18s}.mode:hover{color:var(--text);border-color:#536078}.mode.active{color:#020503;background:var(--green);border-color:var(--green);box-shadow:0 0 28px rgba(0,255,123,.26)}.mode.active.danger{background:var(--red);border-color:var(--red);color:#fff}.mode.active.hyper{background:linear-gradient(135deg,var(--violet),#74e0ff);border-color:var(--violet);color:#06050d}.mode.active.amber{background:var(--amber);border-color:var(--amber);color:#080600}
  .panel{border-left:1px solid var(--border);background:linear-gradient(180deg,rgba(13,16,24,.98),rgba(8,10,16,.98));padding:18px;overflow:auto;display:flex;flex-direction:column;gap:16px}.card{background:linear-gradient(180deg,var(--panel2),rgba(15,18,28,.9));border:1px solid var(--border);border-radius:var(--radius);padding:14px;box-shadow:0 10px 32px rgba(0,0,0,.16)}.section{font-size:10px;font-weight:700;letter-spacing:.11em;text-transform:uppercase;color:var(--muted);margin-bottom:10px;display:flex;justify-content:space-between}.section span{color:var(--green);font-family:'JetBrains Mono'}
  .hero{display:grid;grid-template-columns:1fr 1fr;gap:10px}.metric{background:rgba(255,255,255,.035);border:1px solid rgba(255,255,255,.08);border-radius:14px;padding:13px}.metric .v{font-family:'JetBrains Mono';font-size:24px;font-weight:600;line-height:1;color:var(--green)}.metric .l{font-size:9px;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-top:7px}.metric.red .v{color:var(--red)}.metric.violet .v{color:var(--violet)}.metric.cyan .v{color:var(--cyan)}
  .desc{font-size:12px;line-height:1.55;color:#aeb5c8}.desc strong{color:var(--text)}.sliders{display:flex;flex-direction:column;gap:12px}.slider-label{display:flex;justify-content:space-between;font-size:11px;color:var(--muted);margin-bottom:7px}.slider-label b{font-family:'JetBrains Mono';font-weight:500;color:var(--green)}input[type=range]{appearance:none;width:100%;height:5px;border-radius:99px;background:#252b3a;outline:none}input[type=range]::-webkit-slider-thumb{appearance:none;width:17px;height:17px;border-radius:50%;background:var(--green);box-shadow:0 0 18px rgba(0,255,123,.45);cursor:pointer}.hyper input[type=range]::-webkit-slider-thumb{background:var(--violet);box-shadow:0 0 18px rgba(167,139,250,.45)}
  .submodes{display:flex;flex-wrap:wrap;gap:7px}.submode{font-family:'JetBrains Mono';font-size:9px;text-transform:uppercase;letter-spacing:.07em;background:#161927;color:var(--muted);border:1px solid var(--border);border-radius:10px;padding:8px 9px;cursor:pointer}.submode.active{background:var(--violet);border-color:var(--violet);color:#080611}.readout{display:grid;gap:7px;margin-top:10px}.readout div{display:flex;justify-content:space-between;color:var(--muted);font-size:11px}.readout span{font-family:'JetBrains Mono';color:var(--violet)}
  button,input{font:inherit}button{touch-action:manipulation;-webkit-tap-highlight-color:transparent;user-select:none;-webkit-user-select:none;min-height:44px}.toggle{display:flex;align-items:center;gap:8px;color:var(--muted);font-size:12px;margin-bottom:10px}.btns{display:grid;gap:9px;margin-top:auto}.btn{width:100%;border:1px solid var(--border);border-radius:14px;padding:12px;font-weight:700;font-size:12px;cursor:pointer;transition:.18s;background:#171b27;color:var(--text)}.btn:hover{border-color:#526079}.btn.primary{background:var(--green);border-color:var(--green);color:#031006;box-shadow:0 0 26px rgba(0,255,123,.18)}.btn.cyan{background:#06252b;color:var(--cyan);border-color:rgba(0,231,255,.55)}.btn.cyan.active{background:var(--cyan);color:#021013}.btn.ghost{color:var(--muted)}.tapHint{display:none;margin-top:9px;color:var(--cyan);font-size:11px;line-height:1.4}.flash{position:fixed;inset:0;background:#fff;opacity:0;pointer-events:none;z-index:99;transition:opacity .28s}.flash.show{opacity:.9;transition:opacity .045s}.toast{position:fixed;bottom:22px;left:50%;transform:translateX(-50%) translateY(60px);background:#101522;border:1px solid var(--green);color:var(--green);border-radius:999px;padding:11px 18px;font-size:12px;font-weight:700;opacity:0;z-index:100;box-shadow:0 0 30px rgba(0,255,123,.18);transition:.25s}.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
  .progressTrack{height:7px;border-radius:99px;background:#252b3a;overflow:hidden;margin-top:10px}.progressFill{height:100%;width:0;background:var(--green);transition:width .22s ease}#camPanel,#hyperPanel{display:none}.note{font-size:11px;color:var(--muted);line-height:1.45;margin-top:10px;border-top:1px solid rgba(255,255,255,.08);padding-top:10px}.note b{color:var(--violet)}
  @media(max-width:900px){.topbar{height:54px;padding:0 12px}.brand h1{font-size:13px}.brand small{display:none}.layout{height:auto;min-height:calc(100svh - 54px);display:flex;flex-direction:column}.stage{height:auto;min-height:0;display:block;overflow:visible;padding-top:0}.stage img{width:100%;height:auto;max-height:58svh;min-height:300px;object-fit:contain;background:#000}.panel{border-left:none;border-top:1px solid var(--border);padding:14px 12px calc(104px + env(safe-area-inset-bottom));overflow:visible}.modes{position:relative;left:auto;right:auto;bottom:auto;margin:8px;border-radius:16px;z-index:12;scroll-snap-type:x proximity}.mode{min-height:46px;scroll-snap-align:start}.btns{position:fixed;left:0;right:0;bottom:0;padding:10px 12px calc(10px + env(safe-area-inset-bottom));background:linear-gradient(180deg,rgba(5,6,10,0),rgba(5,6,10,.95) 22%,#090c12);border-top:1px solid var(--border);z-index:30}.btns .ghost{display:none}.toast{bottom:98px;max-width:92vw;text-align:center}.stageHud{top:10px;left:10px;right:10px}.chip{font-size:9px;padding:6px 8px}.card{scroll-margin-top:64px}}
</style>
</head>
<body>
  <div class="topbar">
    <div class="brand"><div class="mark">◉</div><div><h1>Anti-Surveillance Camera</h1><small>reverse surveillance / 4D vision engine</small></div></div>
    <div class="live">LIVE STILL</div>
  </div>
  <div class="layout">
    <main class="stage" id="videoPanel">
      <img id="feed" src="{{ url_for('video_feed') }}" alt="Live camera feed">
      <canvas class="tap-overlay" id="tapCanvas"></canvas>
      <div class="stageHud">
        <div class="chip green" id="modeChip">DATA FIELD</div>
        <button class="chip" id="previewToggle" type="button" onclick="togglePreview()" style="cursor:pointer;background:#172331" title="Fast raw camera vs rendered fine-arts effects">PREVIEW: FAST</button>
        <div class="chip" id="fpsChip">FPS: --</div>
        <div class="chip" id="detectChip">DETECT: --</div>
        <div class="chip violet" id="bgChip">BG: WARMING</div>
      </div>
      <div class="modes" id="modes">
        <button class="mode active" data-mode="env_accurate" onclick="setMode('env_accurate')">Data Field</button>
        <button class="mode" data-mode="env_field" onclick="setMode('env_field')">Field Warp</button>
        <button class="mode" data-mode="env_particles" onclick="setMode('env_particles')">Particles</button>
        <button class="mode" data-mode="env_abstract" onclick="setMode('env_abstract')">Data Abstract</button>
        <button class="mode danger" data-mode="panopticon" onclick="setMode('panopticon')">Panopticon</button>
        <button class="mode" data-mode="ghost" onclick="setMode('ghost')">Ghost</button>
        <button class="mode" data-mode="dissolution" onclick="setMode('dissolution')">Dissolution</button>
        <button class="mode" data-mode="shield" onclick="setMode('shield')">Shield</button>
        <button class="mode" data-mode="erased" onclick="setMode('erased')">Adaptive Erase</button>
        <button class="mode" data-mode="mapped" onclick="setMode('mapped')">Mapped Erase</button>
        <button class="mode" data-mode="hyperspace" onclick="setMode('hyperspace')">4D Hyperspace</button>
        <button class="mode" data-mode="all" onclick="setMode('all')">All Layers</button>
        <button class="mode" data-mode="raw" onclick="setMode('raw')">Raw Camera</button>
      </div>
    </main>
    <aside class="panel">
      <section class="card">
        <div class="section">Detection <span id="detStatus">SEARCHING</span></div>
        <div class="hero">
          <div class="metric red"><div class="v" id="det-count">0</div><div class="l">current subjects</div></div>
          <div class="metric"><div class="v" id="det-total">0</div><div class="l">total detections</div></div>
          <div class="metric cyan"><div class="v" id="space-kp">--</div><div class="l">planetary Kp</div></div>
          <div class="metric violet"><div class="v" id="solar-speed">--</div><div class="l">solar wind km/s</div></div>
        </div>
      </section>
      <section class="card">
        <div class="section">Mode Concept <span id="modeMini">V1</span></div>
        <div class="desc" id="mode-desc"><strong>DATA FIELD</strong> — Exact-location WMM field geometry plus the nearest usable geomagnetic observatory and NOAA space-weather data.</div>
      </section>
      <section class="card" id="effectPanel">
        <div class="section">Effect Engine <span>LIVE</span></div>
        <div class="sliders">
          <div><div class="slider-label"><span>Intensity</span><b id="int-val">78%</b></div><input type="range" id="intensity" min="0" max="100" value="78" oninput="updateFx()"></div>
          <div><div class="slider-label"><span>Persistence</span><b id="gp-val">18</b></div><input type="range" id="ghost_persist" min="2" max="30" value="18" oninput="updateFx()"></div>
          <div><div class="slider-label"><span>Scan / Motion Speed</span><b id="ss-val">3</b></div><input type="range" id="scan_speed" min="1" max="8" value="3" oninput="updateFx()"></div>
        </div>
        <div class="note"><b>Adaptive Erase</b> learns the empty scene automatically. It now replaces people with clean-plate pixels instead of inpainting or blurring them.</div>
        <div class="btns" style="margin-top:10px"><button class="btn ghost" id="imageToneBtn" onclick="toggleTone('image')">Image: B&W</button><button class="btn ghost" id="overlayToneBtn" onclick="toggleTone('overlay')">Overlay: Color</button></div>
      </section>
      <section class="card hyper" id="hyperPanel">
        <div class="section">4D Vision Engine <span>W-AXIS</span></div>
        <div class="submodes">
          <button class="submode active" data-sub="tesseract" onclick="setHyperSub('tesseract')">Tesseract</button>
          <button class="submode" data-sub="clifford" onclick="setHyperSub('clifford')">Clifford</button>
          <button class="submode" data-sub="lissajous" onclick="setHyperSub('lissajous')">Lissajous</button>
          <button class="submode" data-sub="hyperspheres" onclick="setHyperSub('hyperspheres')">Hyperspheres</button>
          <button class="submode" data-sub="all4d" onclick="setHyperSub('all4d')">All W-Slices</button>
        </div>
        <div class="sliders" style="margin-top:12px">
          <div><div class="slider-label"><span>Rotation Speed</span><b id="rotSpeedVal">1.0×</b></div><input type="range" id="rotSpeed" min="0.1" max="4" step="0.1" value="1" oninput="updateHyper()"></div>
          <div><div class="slider-label"><span>W-Depth</span><b id="wDepthVal">2.5</b></div><input type="range" id="wDepth" min="1.5" max="5" step="0.1" value="2.5" oninput="updateHyper()"></div>
        </div>
        <div class="readout"><div>W slice <span id="wSlice">0.000</span></div><div>rotation <span id="rotRad">0.00 rad</span></div><div>entities <span id="entities4d">0</span></div></div>
        <div class="note">A 2D camera cannot literally capture a real 4D world, so this mode builds a cinematic approximation: live camera pixels are rendered as multiple animated <b>W-axis cross-sections</b>, then warped through 4D perspective, hypersphere lensing, and projected geometry.</div>
      </section>
      <section class="card" id="camPanel">
        <div class="section">Camera Controls <span id="camModeLabel">AUTO</span></div>
        <label class="toggle"><input type="checkbox" id="autoExp" checked onchange="updateCam()"> Auto exposure</label>
        <div class="sliders">
          <div id="autoExpCtrl"><div class="slider-label"><span>EV Compensation</span><b id="evVal">+0.0</b></div><input type="range" id="evComp" min="-4" max="4" step="0.1" value="0" oninput="updateCam()"></div>
          <div id="manualExpCtrl" style="display:none"><div class="slider-label"><span>Exposure Time</span><b id="expTimeVal">20ms</b></div><input type="range" id="expTime" min="500" max="200000" step="500" value="20000" oninput="updateCam()"><div class="slider-label" style="margin-top:10px"><span>Analogue Gain</span><b id="isoVal">ISO 100</b></div><input type="range" id="isoGain" min="1" max="16" step="0.1" value="1" oninput="updateCam()"></div>
          <div><div class="slider-label"><span>Brightness</span><b id="camBrightVal">+0.0</b></div><input type="range" id="camBright" min="-1" max="1" step="0.1" value="0" oninput="updateCam()"></div>
          <div><div class="slider-label"><span>Contrast</span><b id="camContrastVal">1.0</b></div><input type="range" id="camContrast" min="0" max="2" step="0.1" value="1" oninput="updateCam()"></div>
          <div><div class="slider-label"><span>Saturation</span><b id="camSatVal">1.0</b></div><input type="range" id="camSat" min="0" max="2" step="0.1" value="1" oninput="updateCam()"></div>
        </div>
      </section>
      <section class="card" id="erasePanel" style="display:none">
        <div class="section">Scene Mapping <span id="mapStatus">NOT MAPPED</span></div>
        <button class="btn primary" type="button" id="scanMapBtn" onclick="scanSceneMap()">Build Clean Scene Map</button>
        <button class="btn ghost" type="button" onclick="clearSceneMap()" style="margin-top:8px">Clear Saved Scene Map</button>
        <div class="progressTrack" aria-label="scene map progress"><div class="progressFill" id="mapProgressBar"></div></div>
        <div class="readout" style="margin-top:10px"><div>phase <span id="mapPhase">IDLE</span></div><div>progress <span id="mapProgress">0/20</span></div><div>accepted / rejected <span id="mapCounts">0 / 0</span></div><div>registration <span id="mapAlignment">IDLE</span></div><div>source <span id="eraseSource">ADAPTIVE</span></div></div>
        <div class="note"><b>Mapped Erase:</b> clear people from the frame and build the map. Small handheld shifts and slight rotation are registered automatically; avoid large reframing until the map reaches READY.</div>
      </section>
      <section class="card">
        <div class="section">Manual Erasure <span>TAP</span></div>
        <button class="btn cyan" id="tapBtn" onclick="toggleTap()">Draw Tap-to-Remove Region</button>
        <button class="btn ghost" onclick="clearTap()" style="margin-top:8px">Clear Manual Regions</button>
        <div class="tapHint" id="tapHint">Drag a rectangle over part of the video. In Erase modes it uses the clean plate; in other modes it uses local inpainting.</div>
      </section>
      <div class="note"><b>Low-bandwidth phone preview / 2K saved stills.</b> Saved JPEGs use the 2560×1440 main stream (1440×2560 after portrait rotation) at high quality. Re-scan Mapped Erase after installing this version to create a native-resolution clean plate.</div>
      <div class="btns"><button class="btn primary" onclick="capture()">⊙ Capture 2K Still</button><button class="btn ghost" onclick="resetAll()">Reset Installation</button></div>
    </aside>
  </div>
  <div class="flash" id="flashOverlay"></div><div class="toast" id="toast"></div>
<script>
const descriptions={
  env_accurate:'<strong>DATA FIELD</strong> — Exact-location WMM vectors, nearest live geomagnetic observatory, OVATION local grid, regional neutron-monitor cosmic rays, and NOAA space-weather data.',
  env_field:'<strong>FIELD WARP</strong> — Magnetic-field direction and live disturbance deform the camera image.',
  env_particles:'<strong>PARTICLE FIELD</strong> — GOES/SWPC/neutron-monitor measurements control event density and streak behavior.',
  env_abstract:'<strong>DATA ABSTRACTION</strong> — Highly abstract deformation driven by the same real measurements.',
  panopticon:'<strong>PANOPTICON</strong> — Institutional gaze and tracking.',ghost:'<strong>GHOST</strong> — Afterimage and background leakage.',dissolution:'<strong>DISSOLUTION</strong> — Identity breaks into unstable data.',shield:'<strong>FIELD SHIELD</strong> — Privacy aura and distortion.',erased:'<strong>ADAPTIVE ERASE</strong> — Learned clean plate.',mapped:'<strong>MAPPED ERASE</strong> — Saved clean scene map.',hyperspace:'<strong>4D HYPERSPACE</strong> — W-axis projection.',all:'<strong>ALL LAYERS</strong> — Existing effects composited.',raw:'<strong>RAW CAMERA</strong> — Camera feed with selected image tone.'
};
const labels={env_accurate:'DATA FIELD',env_field:'FIELD WARP',env_particles:'PARTICLE FIELD',env_abstract:'DATA ABSTRACTION',panopticon:'PANOPTICON',ghost:'GHOST',dissolution:'DISSOLUTION',shield:'SHIELD',erased:'ADAPTIVE ERASE',mapped:'MAPPED ERASE',hyperspace:'4D HYPERSPACE',all:'ALL LAYERS',raw:'RAW CAMERA'};
let imageTone='bw',overlayTone='color';
let currentMode='env_accurate';let previewMode='{{ preview_mode }}';let tapMode=false,tapStart=null,lastCaptureId=null,_pollT=0;const postTimers={};
function showPreviewMode(){document.getElementById('previewToggle').textContent='PREVIEW: '+previewMode.toUpperCase();}
function togglePreview(){const mode=previewMode==='fast'?'fx':'fast';fetch('/preview/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode})}).then(r=>r.json()).then(d=>{previewMode=d.mode;showPreviewMode();toast(previewMode==='fast'?'FAST live view (effects still in captures + TFT)':'FINE ARTS effects live preview (lower FPS)')}).catch(()=>toast('Preview switch failed'));}function postSoon(key,url,payload,delay=120){clearTimeout(postTimers[key]);postTimers[key]=setTimeout(()=>fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}).catch(()=>{}),delay)}
function clsFor(m){return m==='panopticon'?'danger':m==='hyperspace'?'hyper':m==='raw'?'amber':''}
function setMode(m){currentMode=m;document.querySelectorAll('.mode').forEach(b=>{const a=b.dataset.mode===m;b.className='mode'+(a?' active '+clsFor(m):'')});document.getElementById('mode-desc').innerHTML=descriptions[m]||'';document.getElementById('modeChip').textContent=labels[m]||m.toUpperCase();document.getElementById('hyperPanel').style.display=m==='hyperspace'?'block':'none';document.getElementById('camPanel').style.display=m==='raw'?'block':'none';document.getElementById('erasePanel').style.display=(m==='erased'||m==='mapped')?'block':'none';document.getElementById('eraseSource').textContent=m==='mapped'?'MAPPED SCENE':'ADAPTIVE';document.getElementById('effectPanel').style.display=(m==='raw'||m==='hyperspace')?'none':'block';fetch('/viz_settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})});}
function setHyperSub(sub){document.querySelectorAll('.submode').forEach(b=>b.classList.toggle('active',b.dataset.sub===sub));fetch('/hyperspace_settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({submode:sub})});}
function updateHyper(){const rs=parseFloat(document.getElementById('rotSpeed').value),wd=parseFloat(document.getElementById('wDepth').value);document.getElementById('rotSpeedVal').textContent=rs.toFixed(1)+'×';document.getElementById('wDepthVal').textContent=wd.toFixed(1);postSoon('hyper','/hyperspace_settings',{rot_speed:rs,w_depth:wd});}
function updateFx(){const iv=document.getElementById('intensity').value,gp=document.getElementById('ghost_persist').value,ss=document.getElementById('scan_speed').value;document.getElementById('int-val').textContent=iv+'%';document.getElementById('gp-val').textContent=gp;document.getElementById('ss-val').textContent=ss;postSoon('fx','/viz_settings',{intensity:parseInt(iv),ghost_persistence:parseInt(gp),scan_speed:parseInt(ss)});}
function updateCam(){const autoExp=document.getElementById('autoExp').checked,expTime=parseInt(document.getElementById('expTime').value),gain=parseFloat(document.getElementById('isoGain').value),ev=parseFloat(document.getElementById('evComp').value),bright=parseFloat(document.getElementById('camBright').value),contrast=parseFloat(document.getElementById('camContrast').value),sat=parseFloat(document.getElementById('camSat').value);document.getElementById('autoExpCtrl').style.display=autoExp?'block':'none';document.getElementById('manualExpCtrl').style.display=autoExp?'none':'block';document.getElementById('evVal').textContent=(ev>=0?'+':'')+ev.toFixed(1);document.getElementById('expTimeVal').textContent=expTime<1000?expTime+'µs':(expTime/1000).toFixed(0)+'ms';document.getElementById('isoVal').textContent='ISO '+Math.round(gain*100);document.getElementById('camBrightVal').textContent=(bright>=0?'+':'')+bright.toFixed(1);document.getElementById('camContrastVal').textContent=contrast.toFixed(1);document.getElementById('camSatVal').textContent=sat.toFixed(1);document.getElementById('camModeLabel').textContent=autoExp?'AUTO':'MANUAL';postSoon('cam','/cam_controls',{auto_exposure:autoExp,exposure_time:expTime,analogue_gain:gain,ev_compensation:ev,brightness:bright,contrast:contrast,saturation:sat},160);}
function toggleTone(which){if(which==='image'){imageTone=imageTone==='bw'?'color':'bw';document.getElementById('imageToneBtn').textContent='Image: '+(imageTone==='bw'?'B&W':'Color');}else{overlayTone=overlayTone==='color'?'bw':'color';document.getElementById('overlayToneBtn').textContent='Overlay: '+(overlayTone==='bw'?'B&W':'Color');}fetch('/viz_settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({image_tone:imageTone,overlay_tone:overlayTone})});}
function toggleTap(){tapMode=!tapMode;document.getElementById('videoPanel').classList.toggle('tap-mode',tapMode);document.getElementById('tapBtn').classList.toggle('active',tapMode);document.getElementById('tapHint').style.display=tapMode?'block':'none';if(tapMode)toast('Drag on the image to remove an object');}
function coords(e){const img=document.getElementById('feed'),r=img.getBoundingClientRect();return{nx:Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),ny:Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))};}
const tapSurface=document.getElementById('tapCanvas');tapSurface.addEventListener('pointerdown',e=>{if(!tapMode)return;e.preventDefault();tapSurface.setPointerCapture(e.pointerId);tapStart=coords(e)});tapSurface.addEventListener('pointerup',endTap);tapSurface.addEventListener('pointercancel',()=>{tapStart=null});
function endTap(e){if(!tapMode||!tapStart)return;e.preventDefault();const c=coords(e);let nx1=Math.min(tapStart.nx,c.nx),ny1=Math.min(tapStart.ny,c.ny),nx2=Math.max(tapStart.nx,c.nx),ny2=Math.max(tapStart.ny,c.ny);const mn=.08;if(nx2-nx1<mn){const cx=(nx1+nx2)/2;nx1=Math.max(0,cx-mn/2);nx2=Math.min(1,cx+mn/2)}if(ny2-ny1<mn){const cy=(ny1+ny2)/2;ny1=Math.max(0,cy-mn/2);ny2=Math.min(1,cy+mn/2)}fetch('/tap_region',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x1:nx1,y1:ny1,x2:nx2,y2:ny2})}).then(r=>r.json()).then(d=>toast('Manual erasure region added: '+d.total));tapStart=null;}
function clearTap(){fetch('/tap_region/clear',{method:'POST'}).then(()=>toast('Manual erasure regions cleared'));}
function scanSceneMap(){setMode('mapped');fetch('/scene_map/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({frames:20})}).then(r=>r.json()).then(()=>toast('Scene mapping started — clear people; small handheld motion is okay')).catch(()=>toast('Could not start scene scan'));}
function clearSceneMap(){fetch('/scene_map/clear',{method:'POST'}).then(()=>toast('Saved scene map cleared')).catch(()=>toast('Could not clear scene map'));}
function flash(){const f=document.getElementById('flashOverlay');f.classList.add('show');setTimeout(()=>f.classList.remove('show'),70)}
function capture(){flash();fetch('/capture').then(r=>r.json()).then(d=>{if(d.capture_id!==undefined)lastCaptureId=d.capture_id;const dims=(d.width&&d.height)?' '+d.width+'×'+d.height:'';toast(d.success?'✓ 2K capture recorded'+dims+(d.display_name?': '+d.display_name:''):'✗ Capture failed')}).catch(()=>toast('✗ Capture failed'));}
function resetAll(){document.getElementById('intensity').value=78;document.getElementById('ghost_persist').value=18;document.getElementById('scan_speed').value=3;updateFx();setMode('env_accurate');clearTap();}
function toast(msg){const el=document.getElementById('toast');el.textContent=msg;el.classList.add('show');setTimeout(()=>el.classList.remove('show'),2500)}
function poll(){fetch('/detection_data',{cache:'no-store'}).then(r=>r.json()).then(d=>{document.getElementById('fpsChip').textContent='FPS: '+(d.stream_fps||0).toFixed(1);
previewMode=d.preview_mode||previewMode;showPreviewMode();
document.getElementById('det-count').textContent=d.count;document.getElementById('det-total').textContent=d.total_seen;const env=d.environment||{},sw=env.swpc||{};document.getElementById('space-kp').textContent=sw.kp!==null&&sw.kp!==undefined?Number(sw.kp).toFixed(1):'--';document.getElementById('solar-speed').textContent=sw.solar_wind_speed!==null&&sw.solar_wind_speed!==undefined?Math.round(sw.solar_wind_speed):'--';document.getElementById('detStatus').textContent=(d.detection_status||'unknown').toUpperCase().slice(0,18);document.getElementById('detectChip').textContent='DETECT: '+(d.detection_status||'UNKNOWN').toUpperCase().slice(0,14);const mappedReady=!!d.mapped_background_ready,adaptiveReady=!!d.background_ready,target=d.scene_scan_target||20,accepted=d.scene_scan_accepted||0,rejected=d.scene_scan_rejected||0,percent=Math.max(0,Math.min(100,d.scene_scan_percent||0)),phase=(d.scene_scan_phase||'idle').toUpperCase();document.getElementById('bgChip').textContent=currentMode==='mapped'?(mappedReady?'MAP: READY':'MAP: '+accepted+'/'+target):(adaptiveReady?'BG: READY':'BG: '+(d.background_empty_frames||0)+'/'+(d.background_required_frames||12));document.getElementById('mapStatus').textContent=(d.scene_scan_status||'not mapped').toUpperCase().slice(0,34);document.getElementById('mapProgress').textContent=accepted+'/'+target;document.getElementById('mapCounts').textContent=accepted+' / '+rejected;document.getElementById('mapPhase').textContent=phase;document.getElementById('mapAlignment').textContent=(d.scene_scan_alignment||d.alignment_status||'idle').toUpperCase().slice(0,30);document.getElementById('mapProgressBar').style.width=percent+'%';const busy=d.scene_scan_active||phase==='FINALIZING';document.getElementById('scanMapBtn').textContent=phase==='READY'?'Rebuild Clean Scene Map':busy?'Building Scene Map…':'Build Clean Scene Map';document.getElementById('scanMapBtn').disabled=!!busy;if(lastCaptureId===null){lastCaptureId=d.capture_id}else if(d.capture_id!==lastCaptureId){lastCaptureId=d.capture_id;flash();toast(d.capture_success?'✓ Capture recorded'+(d.capture_source==='hardware'?' from shutter':''):'✗ Capture failed')}_pollT+=.03;document.getElementById('wSlice').textContent=Math.sin(_pollT*.38).toFixed(3);document.getElementById('rotRad').textContent=(_pollT%(2*Math.PI)).toFixed(2)+' rad';document.getElementById('entities4d').textContent=d.count;}).catch(()=>{});}poll();setInterval(poll,1000);updateFx();updateCam();
</script>
</body>
</html>
'''

# ─── Routes ───────────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template_string(HTML, preview_mode=_preview_mode)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control':'no-store, no-cache, must-revalidate',
                             'Pragma':'no-cache', 'X-Accel-Buffering':'no'})

@app.route('/preview/mode', methods=['GET', 'POST'])
def preview_mode_route():
    """Switch the web feed FAST <-> FX without restarting the full camera."""
    global _preview_mode, _stream_jpeg
    if request.method == 'POST':
        payload = request.get_json(silent=True) or {}
        mode = str(payload.get('mode', '')).lower()
        if mode not in ('fast', 'fx'):
            return jsonify({'error':'mode must be fast or fx'}), 400
        with _stream_condition:
            _preview_mode = mode
            _stream_jpeg = None
            _stream_stats['mode'] = mode
            _stream_stats['fps'] = 0.0
            _stream_stats['last_frame_monotonic'] = 0.0
            _stream_condition.notify_all()
    return jsonify({'mode':_preview_mode, 'effect_fps_target':PREVIEW_EFFECT_FPS,
                    'fast_fps_target':PREVIEW_FPS})

@app.route('/detection_data')
def get_detection_data():
    env=_environment_snapshot()
    with _capture_lock: cap=dict(capture_state)
    with _detection_lock: det=dict(detection_data); people=list(detection_data.get('people',[]))
    with _background_lock:
        bg_frames=_background_frames; bg_empty_frames=_background_empty_frames; bg_ready=_background_model is not None and bg_empty_frames>=ERASE_BG_READY_EMPTY_FRAMES
    with _scene_map_lock:
        map_ready=_mapped_background is not None; map_created=_mapped_created_at; map_size=None if _mapped_background is None else [int(_mapped_background.shape[1]),int(_mapped_background.shape[0])]; scan_state=dict(_scene_scan_state)
    return jsonify({'count':det.get('count',0),'total_seen':det.get('total_seen',0),'people':[(x1,y1,x2,y2,c) for x1,y1,x2,y2,c in people],'tracks':det.get('tracks',[]),'detection_status':det.get('status','unknown'),'background_ready':bg_ready,'background_frames':bg_frames,'background_empty_frames':bg_empty_frames,'background_required_frames':ERASE_BG_READY_EMPTY_FRAMES,'mapped_background_ready':map_ready,'mapped_background_created_at':map_created,'mapped_background_size':map_size,'preview_size':list(PREVIEW_SIZE),'still_size':list(STILL_SIZE),'scene_scan_active':scan_state.get('active',False),'scene_scan_status':scan_state.get('status','not mapped'),'scene_scan_accepted':scan_state.get('accepted',0),'scene_scan_rejected':scan_state.get('rejected',0),'scene_scan_target':scan_state.get('target',SCENE_SCAN_TARGET),'scene_scan_percent':scan_state.get('percent',0),'scene_scan_phase':scan_state.get('phase','idle'),'scene_scan_alignment':scan_state.get('alignment','idle'),'scene_scan_last_motion':scan_state.get('last_motion'),'alignment_status':_alignment_status.get('status','idle'),'alignment_details':dict(_alignment_status),'stream_fps':_stream_stats.get('fps',0.0),'stream_frame_ms':_stream_stats.get('frame_ms',0.0),'stream_clients':_stream_clients,'preview_fps_target':PREVIEW_FPS,'preview_jpeg_quality':PREVIEW_JPEG_QUALITY,'preview_mode':_preview_mode,'effect_fps_target':PREVIEW_EFFECT_FPS,'lcd_backend':LCD_BACKEND,'environment':env,'capture_id':cap['id'],'capture_success':cap['success'],'capture_name':cap['display_name'],'capture_source':cap['source'],'capture_timestamp':cap['timestamp'],'capture_error':cap['error']})


@app.route('/environment/location', methods=['GET','POST'])
def environment_location_route():
    global camera_location
    if request.method=='GET':
        with _environment_lock: return jsonify(dict(environment_state.get('location',camera_location)))
    data=request.json or {}; lat=_float_or_none(data.get('lat')); lon=_float_or_none(data.get('lon')); alt=_float_or_none(data.get('alt_m',0)) or 0.0
    if lat is None or not -90<=lat<=90: return jsonify({'success':False,'error':'lat must be -90..90'}),400
    if lon is None or not -180<=lon<=180: return jsonify({'success':False,'error':'lon must be -180..180'}),400
    camera_location={'lat':lat,'lon':lon,'alt_m':alt,'source':'runtime'}
    with _environment_lock: environment_state['location']=dict(camera_location)
    try:
        os.makedirs(os.path.dirname(LOCATION_FILE),exist_ok=True)
        with open(LOCATION_FILE,'w',encoding='utf-8') as f: json.dump(camera_location,f,indent=2)
    except Exception as e: print(f"Could not persist location: {e}")
    _geo_export_requested.set()
    threading.Thread(target=_update_environment_once,daemon=True).start(); return jsonify({'success':True,'location':camera_location})

@app.route('/scene_map/start', methods=['POST'])
def scene_map_start_route():
    data = request.json or {}
    state = start_scene_scan(data.get('frames', SCENE_SCAN_TARGET))
    return jsonify({'success': True, 'scan': state})

@app.route('/scene_map/clear', methods=['POST'])
def scene_map_clear_route():
    clear_scene_map()
    return jsonify({'success': True})

@app.route('/viz_settings', methods=['POST'])
def update_viz():
    data=request.json or {}
    if 'mode' in data and str(data['mode']) in LOCAL_MODE_ORDER: viz_settings['mode']=str(data['mode'])
    if 'intensity' in data: viz_settings['intensity']=int(np.clip(int(data['intensity']),0,100))
    if 'ghost_persistence' in data: viz_settings['ghost_persistence']=int(np.clip(int(data['ghost_persistence']),2,30))
    if 'scan_speed' in data: viz_settings['scan_speed']=int(np.clip(int(data['scan_speed']),1,8))
    if str(data.get('image_tone','')).lower() in ('bw','color'): viz_settings['image_tone']=str(data['image_tone']).lower()
    if str(data.get('overlay_tone','')).lower() in ('bw','color'): viz_settings['overlay_tone']=str(data['overlay_tone']).lower()
    return jsonify({'success':True,'settings':viz_settings})

@app.route('/hyperspace_settings', methods=['POST'])
def update_hyperspace():
    data = request.json or {}
    if 'submode' in data:
        sub = str(data['submode'])
        if sub in ('tesseract','clifford','lissajous','hyperspheres','all4d'):
            hyperspace_settings['submode'] = sub
    if 'rot_speed' in data:
        hyperspace_settings['rot_speed'] = float(np.clip(float(data['rot_speed']), 0.1, 4.0))
    if 'w_depth' in data:
        hyperspace_settings['w_depth'] = float(np.clip(float(data['w_depth']), 1.5, 5.0))
    return jsonify({'success': True, 'settings': hyperspace_settings})

@app.route('/cam_controls', methods=['POST'])
def update_cam():
    data = request.json
    with _cam_lock:
        for k in ('auto_exposure','exposure_time','analogue_gain',
                  'ev_compensation','brightness','contrast','saturation'):
            if k in data:
                cam_controls[k] = data[k]
    apply_cam_controls()
    return jsonify({'success': True})

@app.route('/tap_region', methods=['POST'])
def add_tap():
    data = request.json
    with tap_regions_lock:
        tap_regions.append((data['x1'],data['y1'],data['x2'],data['y2']))
    return jsonify({'success':True,'total':len(tap_regions)})

@app.route('/tap_region/clear', methods=['POST'])
def clear_tap():
    with tap_regions_lock: tap_regions.clear()
    return jsonify({'success': True})

@app.route('/capture')
def capture_route():
    result = capture_current_frame(prefix='antisurv', source='web')
    with _capture_lock:
        result['capture_id'] = capture_state['id']
    return jsonify(result)

# ─── GPIO polling threads ────────────────────────────────────────────────────
if HAS_GPIO:
    def do_capture():
        """Capture from physical button with immediate TFT feedback."""
        _hardware_photo_capture()

    def shutter_poll_loop():
        """Poll the configured physical shutter button at 50ms intervals."""
        last = GPIO.HIGH
        while not _shutdown_event.is_set():
            try:
                state = GPIO.input(BUTTON_PIN)
                if last == GPIO.HIGH and state == GPIO.LOW:
                    time.sleep(0.05)  # debounce
                    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                        threading.Thread(target=do_capture, daemon=True).start()
                last = state
            except Exception:
                pass
            time.sleep(0.05)

    threading.Thread(target=shutter_poll_loop, daemon=True).start()
    print(
        f"✓ GPIO polling active — shutter GPIO{BUTTON_PIN}, "
        f"joystick click GPIO{JOYSTICK_SW_PIN}; Pi power controlled by USB-C switch"
    )

# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("\n" + "="*60)
    print("  ANTI-SURVEILLANCE CAMERA v6 — Simplified Local Camera UI")
    print("="*60)
    print("  Access: http://raspberrypi.local:5000")
    print(f"  Shutter button:  GPIO{BUTTON_PIN} (default Pin 13)")
    print(f"  Joystick click:  GPIO{JOYSTICK_SW_PIN} (default Pin 15)")
    print("  I2C bus:          GPIO2/Pin3 SDA + GPIO3/Pin5 SCL — ADS1115 joystick ADC only")
    print("  TFT SPI0:         MOSI GPIO10/Pin19, MISO GPIO9/Pin21, SCK GPIO11/Pin23, CS GPIO8/Pin24")
    print(f"  TFT control:      DC GPIO{LCD_DC_PIN}/Pin22, RESET GPIO{LCD_RESET_PIN}/Pin18")
    print("  Pi power:          external switched USB-C; GPIO17/Pin11 is free")
    print(f"  Capture folder:  {CAPTURE_DIR}")
    print(f"  Preview stream:  {PREVIEW_SIZE[0]}x{PREVIEW_SIZE[1]} @ {PREVIEW_FPS:.0f} fps, JPEG Q{PREVIEW_JPEG_QUALITY} (rotated {PREVIEW_SIZE[1]}x{PREVIEW_SIZE[0]})")
    print(f"  Saved stills:    {STILL_SIZE[0]}x{STILL_SIZE[1]} (rotated output {STILL_SIZE[1]}x{STILL_SIZE[0]}), JPEG Q{STILL_JPEG_QUALITY}")
    print("  Data modes: DATA FIELD, FIELD WARP, PARTICLES, DATA ABSTRACTION")
    print("  Local joystick:    LEFT/RIGHT mode, UP/DOWN intensity; after shutter, LEFT/RIGHT browse, CLICK live")
    print(f"  LCD backend:       {LCD_BACKEND} ILI9340/ILI9341-compatible /dev/spidev0.0")
    print(f"  Web preview:       {_preview_mode.upper()} (switch FAST/FX in web UI); FX target {PREVIEW_EFFECT_FPS:g} fps")
    print(f"  TFT source:        {LCD_SOURCE.upper()} (RAW prioritizes fresh frames; FX shows selected effect)")
    print(f"  Local only:        {LOCAL_ONLY} (web/remote interface disabled)")
    print(f"  Detection:         {'ON' if ENABLE_DETECTION else 'OFF'}")
    print(f"  LCD controller:    {LCD_ROTATION}°; software 180° flip={LCD_SOFTWARE_FLIP_180}; swap565={LCD_SWAP565}")
    print("  LCD diagnostic:    python3 <script> --lcd-test / --lcd-benchmark")
    print("  Joystick test:     python3 <script> --joystick-test")
    print("  Visual engine:     field plates / cloud chamber / emulsion / Faraday veil")
    print("  Image/overlay tone: independent B&W or Color")
    print("="*60 + "\n")
    signal.signal(signal.SIGUSR1, _request_ui_export)
    signal.signal(signal.SIGUSR2, _geo_export_signal)
    threading.Thread(target=_geo_diagnostics_loop,daemon=True,name="geo-json-export").start()
    threading.Thread(target=_geo_poll_loop,daemon=True,name="network-geolocation").start()
    print(f"  GEO: auto={GEO_AUTO}; optional Wi-Fi positioning={bool(GEO_WIFI_API_KEY)}; report={GEO_DIAGNOSTICS_FILE}",flush=True)
    threading.Thread(target=_ui_export_worker, daemon=True, name='ui-png-export').start()
    print(f'  UI export: send SIGUSR1 to this process; output: {UI_EXPORT_DIR}', flush=True)
    _load_mapped_background()
    if ENABLE_DETECTION:
        threading.Thread(target=get_model, daemon=True).start()
    if HAS_LCD:
        threading.Thread(target=local_lcd_loop, daemon=True, name="local-tft").start()
    if HAS_JOYSTICK:
        threading.Thread(target=joystick_loop, daemon=True, name="joystick").start()
    ensure_stream_producer()
    try:
        if LOCAL_ONLY:
            print('✓ LCD-only mode: web server disabled; Ctrl+C stops.', flush=True)
            while not _shutdown_event.is_set():
                _shutdown_event.wait(0.25)
        else:
            app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    finally:
        _shutdown_event.set()
        time.sleep(0.35)
        try:
            picam2.stop()
        except Exception:
            pass
        try:
            picam2.close()
        except Exception:
            pass
        _stop_native_lcd_bridge()
        if HAS_GPIO:
            try:
                GPIO.cleanup()
            except Exception:
                pass
