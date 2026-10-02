import io
import os
import math
import time
import threading
import random
import re
import colorsys
import json
import csv
import urllib.request
import urllib.parse
import cv2
import numpy as np
from datetime import datetime, timezone, timedelta
from flask import Flask, Response, render_template_string, jsonify, request
from picamera2 import Picamera2
from PIL import Image, ImageDraw, ImageFont, ImageEnhance, ImageOps, ImageFilter

SCRIPT_VERSION = "resonance-camera-2026.10-local-radnet-nmdb-goes-v2"

# ─── GPIO Setup ───────────────────────────────────────────────────────────────
try:
    from RPi import GPIO
    import subprocess

    BUTTON_PIN = int(os.environ.get("SHUTTER_PIN", "27"))  # Shutter button — GPIO27 / Pin 13
    # IMPORTANT: GPIO2/Pin 3 = I2C SDA and GPIO3/Pin 5 = I2C SCL.
    # The BMM150 magnetometer uses both lines.  GPIO3 may still be used as a
    # wake-from-halt button, but do not use it as the running script's normal
    # power/exit switch while I2C is active.  Default software power/run switch
    # is therefore GPIO17/Pin 11, wired to GND.
    POWER_PIN  = int(os.environ.get("POWER_PIN", "17"))      # Run/power switch — GPIO17 / Pin 11
    GEIGER_PIN = int(os.environ.get("GEIGER_PIN", "23"))     # Geiger pulse input — GPIO23 / Pin 16

    # Geiger modules should be read as a digital pulse input only.  This script
    # intentionally does not start PWM on GEIGER_PIN; a constant screech is
    # usually the module/buzzer, a stuck signal line, or the wrong pull/edge.
    GEIGER_PULL = os.environ.get("GEIGER_PULL", "UP").upper()       # UP, DOWN, OFF
    GEIGER_EDGE = os.environ.get("GEIGER_EDGE", "FALLING").upper()  # FALLING or RISING
    GEIGER_MIN_PULSE_GAP = float(os.environ.get("GEIGER_MIN_PULSE_GAP", "0.004"))
    GEIGER_DISABLE_EDGE = os.environ.get("GEIGER_DISABLE_EDGE", "0") == "1"

    # Switch behavior: "shutdown" safely powers down the Pi, while "exit"
    # stops only this Flask/Picamera script.  A GPIO switch cannot physically cut
    # 5V power by itself; use a latching power module/HAT for true power cut.
    POWER_ACTION = os.environ.get("POWER_ACTION", "shutdown").lower()  # shutdown or exit
    POWER_HOLD_SEC = float(os.environ.get("POWER_HOLD_SEC", "0.75"))
    DISABLE_POWER_SWITCH = os.environ.get("DISABLE_POWER_SWITCH", "0") == "1"
    ALLOW_POWER_ON_I2C = os.environ.get("ALLOW_POWER_ON_I2C", "0") == "1"
    I2C_GPIO_PINS = {2, 3}
    POWER_PIN_ENABLED = (not DISABLE_POWER_SWITCH) and (POWER_PIN not in I2C_GPIO_PINS or ALLOW_POWER_ON_I2C)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    _PULL_MAP = {
        "UP": GPIO.PUD_UP,
        "DOWN": GPIO.PUD_DOWN,
        "OFF": GPIO.PUD_OFF,
        "NONE": GPIO.PUD_OFF,
    }
    _geiger_pull = _PULL_MAP.get(GEIGER_PULL, GPIO.PUD_UP)

    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    if POWER_PIN_ENABLED:
        GPIO.setup(POWER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    elif POWER_PIN in I2C_GPIO_PINS:
        print(f"⚠ POWER_PIN=GPIO{POWER_PIN} is an I2C SDA/SCL pin used by the BMM150; script polling disabled for that pin.")
        print("  Use GPIO17/Pin 11 for the running power/run switch, or set ALLOW_POWER_ON_I2C=1 only if you are not using I2C.")
    else:
        print("ℹ Software power/run switch disabled by DISABLE_POWER_SWITCH=1.")
    GPIO.setup(GEIGER_PIN, GPIO.IN, pull_up_down=_geiger_pull)

    HAS_GPIO = True
    print(f"✓ GPIO ready. Geiger is INPUT-only on GPIO{GEIGER_PIN}; pull={GEIGER_PULL}, edge={GEIGER_EDGE}.")
except Exception as e:
    print(f"GPIO not available: {e}")
    HAS_GPIO = False

# ─── Magnetometer (BMM150 via I2C) ────────────────────────────────────────────
# Robust BMM150 support.  Prefer the Adafruit driver if installed, otherwise use
# a lightweight smbus2 reader with address scan and safer init timing.
HAS_MAG = False
_mag_driver = None
_mag_bus = None
_mag_sensor = None
BMM150_ADDR = None
MAG_STATUS = {
    "available": False,
    "driver": None,
    "addr": None,
    "error": None,
    "fail_count": 0,
}
_mag_last_heading = None
_mag_lock = threading.Lock()

def _sign_extend(value, bits):
    sign = 1 << (bits - 1)
    return (value & (sign - 1)) - (value & sign)

def _init_magnetometer():
    global HAS_MAG, _mag_driver, _mag_bus, _mag_sensor, BMM150_ADDR

    # Optional higher-level driver path.  If the library is not installed this
    # simply falls back to raw I2C below.
    try:
        import board
        import adafruit_bmm150
        _mag_sensor = adafruit_bmm150.BMM150_I2C(board.I2C())
        _mag_driver = "adafruit_bmm150"
        HAS_MAG = True
        MAG_STATUS.update({"available": True, "driver": _mag_driver, "addr": "auto", "error": None})
        print("✓ BMM150 magnetometer via adafruit_bmm150")
        return
    except Exception as e:
        MAG_STATUS["error"] = f"Adafruit driver unavailable: {e}"

    try:
        import smbus2
        _mag_bus = smbus2.SMBus(1)

        # Bosch's bare BMM150 address can be 0x10-0x13 depending on CSB/SDO;
        # many Raspberry Pi breakout boards default to 0x13.
        for addr in (0x10, 0x11, 0x12, 0x13):
            try:
                chip_id = _mag_bus.read_byte_data(addr, 0x40)
                if chip_id == 0x32:
                    BMM150_ADDR = addr
                    break
            except Exception:
                continue

        if BMM150_ADDR is None:
            raise RuntimeError("BMM150 chip id 0x32 not found at I2C addresses 0x10-0x13")

        # Power on, give the chip enough time to wake, then normal mode.
        _mag_bus.write_byte_data(BMM150_ADDR, 0x4B, 0x01)
        time.sleep(0.05)
        _mag_bus.write_byte_data(BMM150_ADDR, 0x4C, 0x00)  # normal mode
        _mag_bus.write_byte_data(BMM150_ADDR, 0x51, 0x04)  # XY repetitions
        _mag_bus.write_byte_data(BMM150_ADDR, 0x52, 0x0E)  # Z repetitions
        time.sleep(0.05)

        _mag_driver = "smbus2_raw"
        HAS_MAG = True
        MAG_STATUS.update({"available": True, "driver": _mag_driver, "addr": f"0x{BMM150_ADDR:02X}", "error": None})
        print(f"✓ BMM150 magnetometer at 0x{BMM150_ADDR:02X} via smbus2")
    except Exception as e:
        HAS_MAG = False
        MAG_STATUS.update({"available": False, "driver": None, "addr": None, "error": str(e)})
        print(f"Magnetometer not available: {e}")

_init_magnetometer()

def read_magnetometer():
    """Return compass heading in degrees, or None if the sensor is unavailable.

    Note: this is an un-tilt-compensated heading.  Keep the sensor roughly flat,
    and keep it away from the Pi/camera cable, magnets, and speaker/buzzer parts.
    """
    global _mag_last_heading
    if not HAS_MAG:
        return None

    try:
        with _mag_lock:
            if _mag_driver == "adafruit_bmm150" and _mag_sensor is not None:
                x, y, _z = _mag_sensor.magnetic
            else:
                # BMM150 raw data block:
                # X/Y are signed 13-bit values packed across LSB/MSB registers.
                data = _mag_bus.read_i2c_block_data(BMM150_ADDR, 0x42, 8)
                x = _sign_extend((data[1] << 5) | (data[0] >> 3), 13)
                y = _sign_extend((data[3] << 5) | (data[2] >> 3), 13)

            if x == 0 and y == 0:
                raise RuntimeError("zero magnetic vector")

            heading = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
            _mag_last_heading = round(heading, 1)
            MAG_STATUS["fail_count"] = 0
            MAG_STATUS["error"] = None
            return _mag_last_heading
    except Exception as e:
        MAG_STATUS["fail_count"] += 1
        MAG_STATUS["error"] = str(e)
        return None

def magnetometer_poll_loop():
    """Keep a low-rate cached heading so the video renderer never waits on I2C."""
    while True:
        read_magnetometer()
        time.sleep(0.20)

threading.Thread(target=magnetometer_poll_loop, daemon=True, name='magnetometer-poll').start()

# ─── Geiger Counter ───────────────────────────────────────────────────────────
# Rolling 60-second timestamp window for CPM.  GPIO23 is read as an input only;
# no PWM is created here.  If the board screeches continuously when powered,
# check whether the onboard buzzer/alarm is enabled and confirm OUT/INT is wired
# to GPIO23, not the buzzer or HV/power pin.
import collections as _collections
CPM_WINDOW        = 60       # seconds
PULSE_HISTORY_LEN = 1000     # max timestamps to keep
CPM_TO_USVH       = 1.0 / 153.8  # Cajoe RadiationD-v1.1 / J305 tube conversion
GEIGER_STUCK_CPS  = float(os.environ.get("GEIGER_STUCK_CPS", "25"))  # noisy/stuck-line warning

_pulse_times = _collections.deque(maxlen=PULSE_HISTORY_LEN)
_geiger_lock = threading.Lock()
_geiger_last_pulse = 0.0
_geiger_rejected = 0
_geiger_edge_active = False
geiger_data  = {
    'cpm': 0.0,
    'usvh': 0.0,
    'pulses': 0,
    'rejected': 0,
    'status': 'offline' if not HAS_GPIO else 'waiting',
    'line': None,
}

def geiger_pulse(channel=None):
    """Register one Geiger pulse, ignoring impossible chatter/glitches."""
    global _geiger_last_pulse, _geiger_rejected
    now = time.monotonic()
    with _geiger_lock:
        gap = now - _geiger_last_pulse if _geiger_last_pulse else 999.0
        if gap < GEIGER_MIN_PULSE_GAP:
            _geiger_rejected += 1
            geiger_data['rejected'] = _geiger_rejected
            return
        _geiger_last_pulse = now
        _pulse_times.append(now)
        geiger_data['pulses'] += 1
        geiger_data['status'] = 'pulse'

def _geiger_edge_constant():
    if not HAS_GPIO:
        return None
    return GPIO.RISING if GEIGER_EDGE == "RISING" else GPIO.FALLING

if HAS_GPIO:
    try:
        GPIO.remove_event_detect(GEIGER_PIN)  # clear any stale state from previous run
    except Exception:
        pass

    if not GEIGER_DISABLE_EDGE:
        try:
            GPIO.add_event_detect(
                GEIGER_PIN,
                _geiger_edge_constant(),
                callback=geiger_pulse,
                bouncetime=max(1, int(GEIGER_MIN_PULSE_GAP * 1000)),
            )
            _geiger_edge_active = True
            geiger_data['status'] = 'edge'
            print(f"✓ Geiger edge detection active on GPIO{GEIGER_PIN} ({GEIGER_EDGE}, input-only; no PWM)")
        except RuntimeError as e:
            geiger_data['status'] = 'poll'
            print(f"Geiger edge detect failed ({e}) — using poll fallback")
    else:
        geiger_data['status'] = 'poll'
        print("Geiger edge detection disabled by GEIGER_DISABLE_EDGE=1 — using poll fallback")

def geiger_poll_loop():
    """Fallback sampler for Geiger modules/OS builds where edge detection fails."""
    if not HAS_GPIO or _geiger_edge_active:
        return
    try:
        last = GPIO.input(GEIGER_PIN)
    except Exception:
        return

    while True:
        try:
            state = GPIO.input(GEIGER_PIN)
            falling = (last == GPIO.HIGH and state == GPIO.LOW)
            rising  = (last == GPIO.LOW  and state == GPIO.HIGH)
            if (GEIGER_EDGE == "RISING" and rising) or (GEIGER_EDGE != "RISING" and falling):
                geiger_pulse(GEIGER_PIN)
            last = state
        except Exception:
            pass
        time.sleep(0.001)

def geiger_cpm_loop():
    """Update CPM every second using a rolling 60-second window."""
    while True:
        time.sleep(1)
        now    = time.monotonic()
        cutoff = now - CPM_WINDOW
        short_cutoff = now - 5.0
        with _geiger_lock:
            recent  = [t for t in _pulse_times if t >= cutoff]
            recent5 = [t for t in _pulse_times if t >= short_cutoff]
            elapsed = min(now - _pulse_times[0], CPM_WINDOW) if _pulse_times else CPM_WINDOW
            rejected = _geiger_rejected
        if elapsed < 1:
            elapsed = 1

        cpm  = round(len(recent) / elapsed * 60.0, 1)
        usvh = round(cpm * CPM_TO_USVH, 4)
        cps5 = len(recent5) / 5.0

        line_state = None
        if HAS_GPIO:
            try:
                line_state = GPIO.input(GEIGER_PIN)
            except Exception:
                line_state = None

        status = 'ok'
        if not HAS_GPIO:
            status = 'offline'
        elif cps5 >= GEIGER_STUCK_CPS:
            status = 'noisy/stuck'
        elif len(recent) == 0:
            status = 'waiting'
        elif rejected > 0:
            status = 'filtered'

        with _geiger_lock:
            geiger_data['cpm']      = cpm
            geiger_data['usvh']     = usvh
            geiger_data['line']     = line_state
            geiger_data['rejected'] = rejected
            geiger_data['status']   = status

threading.Thread(target=geiger_poll_loop, daemon=True).start()
threading.Thread(target=geiger_cpm_loop, daemon=True).start()

# ─── Networked Environmental Data ─────────────────────────────────────────────
# Network I/O stays off the render thread. A background worker refreshes four
# complementary layers and stores only compact cached snapshots:
#   1) USGS ground geomagnetism
#   2) EPA RadNet local/regional environmental gamma radiation
#   3) NMDB neutron-monitor cosmic-ray counts
#   4) NOAA GOES proton/electron flux in near-Earth space
# The live renderer only reads this cache, so a slow API cannot stall the camera.
ENV_REFRESH_SEC = float(os.environ.get("ENV_REFRESH_SEC", "60"))
RADNET_REFRESH_SEC = float(os.environ.get("RADNET_REFRESH_SEC", "300"))
NMDB_REFRESH_SEC = float(os.environ.get("NMDB_REFRESH_SEC", "120"))
ENV_HTTP_TIMEOUT = float(os.environ.get("ENV_HTTP_TIMEOUT", "5.0"))
AUTO_IP_LOCATION = os.environ.get("AUTO_IP_LOCATION", "1") == "1"
CAMERA_LAT = os.environ.get("CAMERA_LAT")
CAMERA_LON = os.environ.get("CAMERA_LON")
CAMERA_LOCATION_NAME = os.environ.get("CAMERA_LOCATION_NAME", "")
CAMERA_CITY = os.environ.get("CAMERA_CITY", "")
CAMERA_STATE = os.environ.get("CAMERA_STATE", "").upper()
GEOMAG_STATION_OVERRIDE = os.environ.get("GEOMAG_STATION", "").strip().upper()
RADNET_CITY_OVERRIDE = os.environ.get("RADNET_CITY", "").strip()
RADNET_STATE_OVERRIDE = os.environ.get("RADNET_STATE", "").strip().upper()
NMDB_STATION_OVERRIDE = os.environ.get("NMDB_STATION", "").strip().upper()
LOCATION_REFRESH_SEC = float(os.environ.get("LOCATION_REFRESH_SEC", "1800"))

# USGS observatory coordinates. Used only to choose a compatible station; actual
# values come from the USGS Geomagnetism Web Service.
USGS_GEOMAG_STATIONS = {
    'BOU': {'name': 'Boulder, CO', 'lat': 40.1375, 'lon': -105.2372},
    'BRW': {'name': 'Barrow, AK', 'lat': 71.3226, 'lon': -156.6231},
    'BSL': {'name': 'Stennis Space Center, MS', 'lat': 30.3504, 'lon': -89.6351},
    'CMO': {'name': 'College, AK', 'lat': 64.8742, 'lon': -147.8597},
    'DED': {'name': 'Deadhorse, AK', 'lat': 70.3552, 'lon': -148.7928},
    'FRD': {'name': 'Fredericksburg, VA', 'lat': 38.2047, 'lon': -77.3729},
    'FRN': {'name': 'Fresno, CA', 'lat': 37.0913, 'lon': -119.7193},
    'GUA': {'name': 'Guam', 'lat': 13.5894, 'lon': 144.8694},
    'HON': {'name': 'Honolulu, HI', 'lat': 21.3166, 'lon': -158.0001},
    'NEW': {'name': 'Newport, WA', 'lat': 48.2649, 'lon': -117.1231},
    'SHU': {'name': 'Shumagin, AK', 'lat': 55.3472, 'lon': -160.4645},
    'SIT': {'name': 'Sitka, AK', 'lat': 57.0576, 'lon': -135.3273},
    'SJG': {'name': 'San Juan, PR', 'lat': 18.1110, 'lon': -66.1500},
    'TUC': {'name': 'Tucson, AZ', 'lat': 32.1742, 'lon': -110.7337},
}

# RadNet monitors are identified by city/state in the public CSV service. EPA
# notes historical station coordinates are often city-centroid based. This
# registry prioritizes the Northeast / current Boston test region and several
# common US locations; exact city matches always win, and overrides are exposed
# for any station not listed here.
RADNET_STATIONS = [
    {'city':'Boston','state':'MA','lat':42.3601,'lon':-71.0589},
    {'city':'Worcester','state':'MA','lat':42.2626,'lon':-71.8023},
    {'city':'Providence','state':'RI','lat':41.8240,'lon':-71.4128},
    {'city':'Portsmouth','state':'NH','lat':43.0718,'lon':-70.7626},
    {'city':'Concord','state':'NH','lat':43.2081,'lon':-71.5376},
    {'city':'Portland','state':'ME','lat':43.6591,'lon':-70.2568},
    {'city':'Orono','state':'ME','lat':44.8831,'lon':-68.6719},
    {'city':'Burlington','state':'VT','lat':44.4759,'lon':-73.2121},
    {'city':'Albany','state':'NY','lat':42.6526,'lon':-73.7562},
    {'city':'New York City','state':'NY','lat':40.7128,'lon':-74.0060},
    {'city':'Yaphank','state':'NY','lat':40.8368,'lon':-72.9170},
    {'city':'Edison','state':'NJ','lat':40.5187,'lon':-74.4121},
    {'city':'Philadelphia','state':'PA','lat':39.9526,'lon':-75.1652},
    {'city':'Baltimore','state':'MD','lat':39.2904,'lon':-76.6122},
    {'city':'Washington','state':'DC','lat':38.9072,'lon':-77.0369},
    {'city':'Richmond','state':'VA','lat':37.5407,'lon':-77.4360},
    {'city':'Virginia Beach','state':'VA','lat':36.8529,'lon':-75.9780},
    {'city':'Charlotte','state':'NC','lat':35.2271,'lon':-80.8431},
    {'city':'Raleigh','state':'NC','lat':35.7796,'lon':-78.6382},
    {'city':'Atlanta','state':'GA','lat':33.7490,'lon':-84.3880},
    {'city':'Miami','state':'FL','lat':25.7617,'lon':-80.1918},
    {'city':'Chicago','state':'IL','lat':41.8781,'lon':-87.6298},
    {'city':'Detroit','state':'MI','lat':42.3314,'lon':-83.0458},
    {'city':'St. Louis','state':'MO','lat':38.6270,'lon':-90.1994},
    {'city':'Dallas','state':'TX','lat':32.7767,'lon':-96.7970},
    {'city':'Austin','state':'TX','lat':30.2672,'lon':-97.7431},
    {'city':'Denver','state':'CO','lat':39.7392,'lon':-104.9903},
    {'city':'Albuquerque','state':'NM','lat':35.0844,'lon':-106.6504},
    {'city':'Las Vegas','state':'NV','lat':36.1699,'lon':-115.1398},
    {'city':'Reno','state':'NV','lat':39.5296,'lon':-119.8138},
    {'city':'Seattle','state':'WA','lat':47.6062,'lon':-122.3321},
    {'city':'Portland','state':'OR','lat':45.5152,'lon':-122.6784},
    {'city':'Honolulu','state':'HI','lat':21.3099,'lon':-157.8581},
    {'city':'Anchorage','state':'AK','lat':61.2181,'lon':-149.9003},
]

# A compact North-American NMDB registry is enough to make the cosmic-ray layer
# geographically meaningful without querying every world station each cycle.
# Newark/Swarthmore (NEWK) is normally the closest NMDB monitor to Boston.
NMDB_STATIONS = {
    'NEWK': {'name':'Newark/Swarthmore, USA', 'lat':39.68, 'lon':-75.75},
    'CALG': {'name':'Calgary, Canada', 'lat':51.08, 'lon':-114.13},
    'FSMT': {'name':'Fort Smith, Canada', 'lat':60.02, 'lon':-111.93},
    'INVK': {'name':'Inuvik, Canada', 'lat':68.36, 'lon':-133.72},
    'NAIN': {'name':'Nain, Canada', 'lat':56.55, 'lon':-61.68},
    'MXCO': {'name':'Mexico City, Mexico', 'lat':19.33, 'lon':-99.18},
}

_env_lock = threading.Lock()
environment_data = {
    'status': 'starting', 'updated': None, 'error': None,
    'location': {'lat': None, 'lon': None, 'name': None, 'city':None, 'state':None, 'source': None},
    'geomag': {
        'available': False, 'station': None, 'station_name': None, 'distance_km': None,
        'time': None, 'x': None, 'y': None, 'z': None, 'f': None,
        'horizontal': None, 'declination_deg': None, 'inclination_deg': None,
        'variation_nt': 0.0, 'activity': 0.0, 'source': 'USGS Geomagnetism',
    },
    'radiation': {
        'available': False, 'station': None, 'distance_km': None, 'time': None,
        'gamma_mean': None, 'exposure_rate': None, 'activity': 0.0,
        'source': 'EPA RadNet', 'note': 'local/regional station measurement',
    },
    'cosmic': {
        'available': False, 'station': None, 'station_name': None, 'distance_km': None,
        'time': None, 'count_rate': None, 'relative_percent': None, 'activity': 0.0,
        'source': 'NMDB', 'note': 'regional secondary cosmic-ray neutron monitor',
    },
    'particles': {
        'available': False, 'time': None, 'satellite': None,
        'proton_10': 0.0, 'proton_50': 0.0, 'proton_100': 0.0,
        'electron': 0.0, 'activity': 0.0, 'source': 'NOAA SWPC GOES',
        'note': 'near-Earth space environment; not local to the camera',
    },
}
_location_cache = {'ts': 0.0, 'value': None}
_source_cache = {'radnet_ts':0.0, 'radnet':None, 'nmdb_ts':0.0, 'nmdb':None}


def _http_bytes(url, timeout=ENV_HTTP_TIMEOUT, accept='*/*'):
    req = urllib.request.Request(url, headers={
        'User-Agent': 'ResonanceCamera/2.0 (+https://github.com/CJD-11/Resonance-Camera)',
        'Accept': accept,
    })
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _http_json(url, timeout=ENV_HTTP_TIMEOUT):
    return json.loads(_http_bytes(url, timeout, 'application/json').decode('utf-8', errors='replace'))


def _http_text(url, timeout=ENV_HTTP_TIMEOUT):
    return _http_bytes(url, timeout, 'text/plain,text/csv,*/*').decode('utf-8-sig', errors='replace')


def _haversine_km(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2-lat1); dl = math.radians(lon2-lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 6371.0088 * 2 * math.asin(min(1.0, math.sqrt(a)))


def _resolve_camera_location():
    global _location_cache
    if CAMERA_LAT is not None and CAMERA_LON is not None:
        try:
            return {'lat':float(CAMERA_LAT),'lon':float(CAMERA_LON),
                    'name':CAMERA_LOCATION_NAME or ', '.join(v for v in (CAMERA_CITY,CAMERA_STATE) if v) or 'configured location',
                    'city':CAMERA_CITY or None,'state':CAMERA_STATE or None,'source':'environment'}
        except Exception:
            pass
    now=time.monotonic()
    if _location_cache['value'] is not None and now-_location_cache['ts'] < LOCATION_REFRESH_SEC:
        return dict(_location_cache['value'])
    if AUTO_IP_LOCATION:
        for url in ('https://ipwho.is/','https://ipapi.co/json/'):
            try:
                d=_http_json(url, timeout=min(ENV_HTTP_TIMEOUT,3.5))
                lat=d.get('latitude',d.get('lat')); lon=d.get('longitude',d.get('lon'))
                if lat is None or lon is None: continue
                city=d.get('city') or ''
                region=d.get('region') or d.get('region_name') or ''
                state=d.get('region_code') or d.get('state_code') or ''
                name=', '.join(v for v in (city,region) if v) or 'IP-estimated region'
                loc={'lat':float(lat),'lon':float(lon),'name':name,'city':city or None,'state':str(state).upper() or None,'source':'ip'}
                _location_cache={'ts':now,'value':loc}; return dict(loc)
            except Exception:
                continue
    return {'lat':None,'lon':None,'name':'location unavailable','city':None,'state':None,'source':None}


def _nearest_from_registry(location, registry, mapping=False):
    lat,lon=location.get('lat'),location.get('lon')
    if lat is None or lon is None: return None,None
    items=registry.items() if mapping else [(None,x) for x in registry]
    ranked=[]
    for key,meta in items:
        d=_haversine_km(float(lat),float(lon),float(meta['lat']),float(meta['lon']))
        ranked.append((d,key,meta))
    ranked.sort(key=lambda x:x[0]); return ranked[0][1:] if ranked else (None,None)


def _nearest_geomag_station(lat, lon):
    code_meta=_nearest_from_registry({'lat':lat,'lon':lon},USGS_GEOMAG_STATIONS,True)
    code,meta=code_meta
    if not code: return None,None
    return code,_haversine_km(float(lat),float(lon),meta['lat'],meta['lon'])


def _valid_number(v):
    try:
        x=float(str(v).replace(',','').strip())
        return x if np.isfinite(x) and abs(x)<1e12 else None
    except Exception:return None


def _fetch_usgs_geomag(location):
    if GEOMAG_STATION_OVERRIDE:
        code=GEOMAG_STATION_OVERRIDE; meta=USGS_GEOMAG_STATIONS.get(code)
        distance=_haversine_km(float(location['lat']),float(location['lon']),meta['lat'],meta['lon']) if meta and location.get('lat') is not None else 0.0
    else:
        code,distance=_nearest_geomag_station(location.get('lat'),location.get('lon'))
    if not code: raise RuntimeError('camera location unavailable; set CAMERA_LAT/CAMERA_LON or GEOMAG_STATION')
    end=datetime.now(timezone.utc); start=end-timedelta(minutes=24)
    q=urllib.parse.urlencode({'id':code,'elements':'X,Y,Z,F','sampling_period':60,'type':'variation','format':'json',
        'starttime':start.strftime('%Y-%m-%dT%H:%M:%SZ'),'endtime':end.strftime('%Y-%m-%dT%H:%M:%SZ')})
    data=_http_json('https://geomag.usgs.gov/ws/data/?'+q)
    times=data.get('times') or []; channels={}; histories={}
    for item in data.get('values',[]):
        meta=item.get('metadata') or {}; el=str(meta.get('element') or meta.get('channel') or '').upper()
        vals=[_valid_number(v) for v in (item.get('values') or [])]; clean=[v for v in vals if v is not None]
        if el and clean: channels[el]=clean[-1]; histories[el]=clean[-20:]
    x,y,z,f=(channels.get(k) for k in ('X','Y','Z','F'))
    if x is None and y is None and z is None and f is None: raise RuntimeError(f'no usable geomagnetic values for {code}')
    horizontal=math.hypot(x or 0.0,y or 0.0) if (x is not None or y is not None) else None
    decl=(math.degrees(math.atan2(y,x))+360.0)%360.0 if x is not None and y is not None else None
    incl=math.degrees(math.atan2(z,max(horizontal or 0.0,1e-6))) if z is not None and horizontal is not None else None
    variances=[]
    for k in ('X','Y','Z','F'):
        vals=histories.get(k,[])
        if len(vals)>=2: variances.append(float(np.nanmax(vals)-np.nanmin(vals)))
    variation=max(variances) if variances else 0.0
    activity=float(np.clip(math.log1p(max(0.0,variation))/math.log(151.0),0.0,1.0))
    return {'available':True,'station':code,'station_name':USGS_GEOMAG_STATIONS.get(code,{}).get('name',code),
        'distance_km':round(float(distance),1),'time':times[-1] if times else end.isoformat(),'x':x,'y':y,'z':z,'f':f,
        'horizontal':horizontal,'declination_deg':decl,'inclination_deg':incl,'variation_nt':variation,'activity':activity,'source':'USGS Geomagnetism'}


def _choose_radnet_station(location):
    if RADNET_CITY_OVERRIDE and RADNET_STATE_OVERRIDE:
        for st in RADNET_STATIONS:
            if st['city'].lower()==RADNET_CITY_OVERRIDE.lower() and st['state']==RADNET_STATE_OVERRIDE:
                d=_haversine_km(float(location.get('lat') or st['lat']),float(location.get('lon') or st['lon']),st['lat'],st['lon'])
                return st,d
        return {'city':RADNET_CITY_OVERRIDE,'state':RADNET_STATE_OVERRIDE,'lat':location.get('lat'),'lon':location.get('lon')},0.0
    city=(location.get('city') or '').lower(); state=(location.get('state') or '').upper()
    for st in RADNET_STATIONS:
        if city and st['city'].lower()==city and (not state or st['state']==state): return st,0.0
    _,st=_nearest_from_registry(location,RADNET_STATIONS,False)
    if not st: return None,None
    d=_haversine_km(float(location['lat']),float(location['lon']),st['lat'],st['lon'])
    return st,d


def _parse_radnet_csv(text):
    # EPA files can contain metadata/header rows. Find the first row that looks
    # like a real CSV header, then parse numeric gamma/exposure columns robustly.
    lines=[ln for ln in text.splitlines() if ln.strip()]
    if not lines: raise RuntimeError('empty RadNet response')
    header_idx=0
    for i,ln in enumerate(lines[:40]):
        low=ln.lower()
        if ',' in ln and any(k in low for k in ('date','time','gamma','exposure','location')):
            header_idx=i; break
    sample='\n'.join(lines[header_idx:header_idx+5])
    try: dialect=csv.Sniffer().sniff(sample, delimiters=',;\t|')
    except Exception: dialect=csv.excel
    rows=list(csv.DictReader(lines[header_idx:],dialect=dialect))
    if not rows: raise RuntimeError('no RadNet rows parsed')
    # Keep rows containing at least one numeric measurement.
    usable=[]
    for row in rows:
        nums={k:_valid_number(v) for k,v in row.items() if k is not None}
        vals=[v for v in nums.values() if v is not None]
        if vals: usable.append((row,nums))
    if not usable: raise RuntimeError('RadNet response contains no numeric measurements')
    recent=usable[-24:]; row,nums=usable[-1]
    headers=[str(k or '') for k in row.keys()]
    gamma_keys=[k for k in headers if any(t in k.lower() for t in ('gamma','gross count','cpm')) and nums.get(k) is not None]
    exp_keys=[k for k in headers if 'exposure' in k.lower() and nums.get(k) is not None]
    # Fall back to numeric measurement columns while excluding obvious IDs/dates.
    if not gamma_keys:
        gamma_keys=[k for k in headers if nums.get(k) is not None and not any(t in k.lower() for t in ('year','month','day','hour','minute','latitude','longitude','id'))]
    gamma_vals=[nums[k] for k in gamma_keys if nums.get(k) is not None]
    gamma_mean=float(np.mean(gamma_vals)) if gamma_vals else None
    exposure=nums.get(exp_keys[0]) if exp_keys else None
    # Relative short-window deviation is more robust across stations/channels than
    # imposing one national absolute threshold.
    hist=[]
    for rr,nn in recent:
        vs=[nn.get(k) for k in gamma_keys if nn.get(k) is not None]
        if vs: hist.append(float(np.mean(vs)))
    baseline=float(np.median(hist)) if hist else gamma_mean
    rel=abs((gamma_mean or baseline or 0.0)-(baseline or 0.0))/max(abs(baseline or 0.0),1e-6)
    activity=float(np.clip(rel*7.5,0.0,1.0))
    time_value=None
    for k,v in row.items():
        if k and any(t in k.lower() for t in ('date','time')) and str(v).strip():
            time_value=str(v).strip(); break
    return gamma_mean,exposure,activity,time_value,list(gamma_keys)[:12]


def _fetch_epa_radnet(location):
    st,distance=_choose_radnet_station(location)
    if not st: raise RuntimeError('no RadNet station available for location')
    year=datetime.now(timezone.utc).year
    city_path=urllib.parse.quote(st['city'].upper(),safe='')
    url=f"https://radnet.epa.gov/cdx-radnet-rest/api/rest/csv/{year}/fixed/{st['state']}/{city_path}"
    text=_http_text(url, timeout=max(ENV_HTTP_TIMEOUT,7.0))
    gamma,exposure,activity,timestamp,channels=_parse_radnet_csv(text)
    return {'available':True,'station':f"{st['city']}, {st['state']}",'distance_km':round(float(distance or 0.0),1),
        'time':timestamp,'gamma_mean':gamma,'exposure_rate':exposure,'activity':activity,'channels':channels,
        'source':'EPA RadNet','url':url,'note':'local/regional station measurement; not measured at the subject'}


def _choose_nmdb_station(location):
    if NMDB_STATION_OVERRIDE:
        meta=NMDB_STATIONS.get(NMDB_STATION_OVERRIDE)
        d=_haversine_km(float(location['lat']),float(location['lon']),meta['lat'],meta['lon']) if meta and location.get('lat') is not None else 0.0
        return NMDB_STATION_OVERRIDE,meta or {'name':NMDB_STATION_OVERRIDE},d
    code,meta=_nearest_from_registry(location,NMDB_STATIONS,True)
    if not code:return None,None,None
    return code,meta,_haversine_km(float(location['lat']),float(location['lon']),meta['lat'],meta['lon'])


def _parse_nmdb_ascii(text):
    vals=[]; timestamps=[]
    for ln in text.splitlines():
        line=ln.strip()
        if not line or line.startswith('#') or line.startswith('<'): continue
        parts=[p.strip() for p in re.split(r'[;\t, ]+',line) if p.strip()]
        nums=[]
        for p in parts:
            v=_valid_number(p)
            if v is not None: nums.append(v)
        if not nums: continue
        # Count rate is conventionally the final numeric column in NEST ASCII.
        vals.append(float(nums[-1])); timestamps.append(parts[0] if parts else '')
    if not vals: raise RuntimeError('no NMDB count values parsed')
    recent=vals[-60:]; current=recent[-1]; baseline=float(np.median(recent))
    rel_pct=((current-baseline)/max(abs(baseline),1e-9))*100.0
    activity=float(np.clip(abs(rel_pct)/5.0,0.0,1.0))
    return current,rel_pct,activity,(timestamps[-1] if timestamps else None)


def _fetch_nmdb_cosmic(location):
    code,meta,distance=_choose_nmdb_station(location)
    if not code: raise RuntimeError('camera location unavailable; set CAMERA_LAT/CAMERA_LON or NMDB_STATION')
    q=urllib.parse.urlencode([
        ('formchk','1'),('stations[]',code),('output','ascii'),('tabchoice','revori'),
        ('dtype','corr_for_efficiency'),('date_choice','last'),('last_days','180'),
        ('last_label','mins_label'),('tresolution','5'),('yunits','0')
    ])
    url='https://www.nmdb.eu/nest/draw_graph.php?'+q
    text=_http_text(url, timeout=max(ENV_HTTP_TIMEOUT,7.0))
    count,rel,activity,timestamp=_parse_nmdb_ascii(text)
    return {'available':True,'station':code,'station_name':meta.get('name',code),'distance_km':round(float(distance),1),
        'time':timestamp,'count_rate':count,'relative_percent':rel,'activity':activity,'source':'NMDB',
        'url':url,'note':'regional secondary cosmic-ray neutron monitor; not a personal detector'}


def _latest_flux_by_energy(rows):
    latest={}; latest_time=None; satellite=None
    if not isinstance(rows,list):return latest,latest_time,satellite
    for row in rows:
        if not isinstance(row,dict):continue
        flux=_valid_number(row.get('flux'))
        if flux is None:continue
        energy=str(row.get('energy') or row.get('energy_channel') or row.get('channel') or '').strip()
        t=str(row.get('time_tag') or row.get('time') or '')
        prev=latest.get(energy)
        if prev is None or t>=prev[0]: latest[energy]=(t,max(0.0,flux))
        if t and (latest_time is None or t>latest_time): latest_time=t; satellite=row.get('satellite',satellite)
    return {k:v[1] for k,v in latest.items()},latest_time,satellite


def _pick_energy_value(mapping,target):
    target=float(target); best=None
    for label,value in mapping.items():
        nums=re.findall(r'[-+]?\d*\.?\d+',str(label))
        if not nums:continue
        try:e=float(nums[0])
        except Exception:continue
        diff=abs(e-target)
        if best is None or diff<best[0]:best=(diff,value)
    return float(best[1]) if best else 0.0


def _fetch_noaa_particles():
    protons,pt,sat=_latest_flux_by_energy(_http_json('https://services.swpc.noaa.gov/json/goes/primary/integral-protons-6-hour.json'))
    electrons,et,esat=_latest_flux_by_energy(_http_json('https://services.swpc.noaa.gov/json/goes/primary/integral-electrons-6-hour.json'))
    p10=_pick_energy_value(protons,10); p50=_pick_energy_value(protons,50); p100=_pick_energy_value(protons,100); electron=_pick_energy_value(electrons,2)
    pa=float(np.clip((math.log10(max(p10,1e-5))+5.0)/7.0,0.0,1.0)); ea=float(np.clip((math.log10(max(electron,1e-2))+2.0)/7.0,0.0,1.0))
    return {'available':bool(protons or electrons),'time':max([t for t in (pt,et) if t] or [datetime.now(timezone.utc).isoformat()]),
        'satellite':sat or esat,'proton_10':p10,'proton_50':p50,'proton_100':p100,'electron':electron,
        'activity':max(pa,ea*0.65),'source':'NOAA SWPC GOES','note':'near-Earth space environment; not local to the subject'}


def get_environment_snapshot():
    with _env_lock:
        return {k:(dict(v) if isinstance(v,dict) else v) for k,v in environment_data.items()}


def environmental_data_loop():
    while True:
        cycle_start=time.monotonic(); loc=_resolve_camera_location(); errors=[]
        geomag=radiation=cosmic=particles=None
        try: geomag=_fetch_usgs_geomag(loc)
        except Exception as e: errors.append('USGS: '+str(e))
        # RadNet and NMDB update more slowly; retain cached values between polls.
        now=time.monotonic()
        try:
            if _source_cache['radnet'] is None or now-_source_cache['radnet_ts']>=RADNET_REFRESH_SEC:
                _source_cache['radnet']=_fetch_epa_radnet(loc); _source_cache['radnet_ts']=now
            radiation=_source_cache['radnet']
        except Exception as e: errors.append('EPA RadNet: '+str(e)); radiation=_source_cache.get('radnet')
        try:
            if _source_cache['nmdb'] is None or now-_source_cache['nmdb_ts']>=NMDB_REFRESH_SEC:
                _source_cache['nmdb']=_fetch_nmdb_cosmic(loc); _source_cache['nmdb_ts']=now
            cosmic=_source_cache['nmdb']
        except Exception as e: errors.append('NMDB: '+str(e)); cosmic=_source_cache.get('nmdb')
        try: particles=_fetch_noaa_particles()
        except Exception as e: errors.append('NOAA: '+str(e))
        with _env_lock:
            environment_data['location']=loc
            if geomag is not None: environment_data['geomag']=geomag
            if radiation is not None: environment_data['radiation']=radiation
            if cosmic is not None: environment_data['cosmic']=cosmic
            if particles is not None: environment_data['particles']=particles
            environment_data['updated']=datetime.now(timezone.utc).isoformat(); environment_data['error']=' | '.join(errors) if errors else None
            live_layers=sum(bool((environment_data.get(k) or {}).get('available')) for k in ('geomag','radiation','cosmic','particles'))
            environment_data['status']='live' if live_layers>=3 and not errors else ('partial' if live_layers else 'offline')
        elapsed=time.monotonic()-cycle_start; time.sleep(max(5.0,ENV_REFRESH_SEC-elapsed))


threading.Thread(target=environmental_data_loop, daemon=True, name='environment-api-poll').start()

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
    int(os.environ.get("PREVIEW_WIDTH", "640")),
    int(os.environ.get("PREVIEW_HEIGHT", "360")),
)
STILL_SIZE = (
    int(os.environ.get("STILL_WIDTH", "2560")),
    int(os.environ.get("STILL_HEIGHT", "1440")),
)
STILL_JPEG_QUALITY = int(np.clip(int(os.environ.get("STILL_JPEG_QUALITY", "96")), 80, 100))
PREVIEW_JPEG_QUALITY = int(np.clip(int(os.environ.get("PREVIEW_JPEG_QUALITY", "64")), 45, 85))
PREVIEW_FPS = float(np.clip(float(os.environ.get("PREVIEW_FPS", "30")), 10.0, 40.0))
PREVIEW_FRAME_MAX_US = int(np.clip(int(os.environ.get("PREVIEW_FRAME_MAX_US", "33333")), 20000, 100000))
DETECT_FPS = float(np.clip(float(os.environ.get("DETECT_FPS", "5")), 1.0, 10.0))
BACKGROUND_LEARN_EVERY = int(np.clip(int(os.environ.get("BACKGROUND_LEARN_EVERY", "10")), 1, 30))
SCENE_SCAN_EVERY = int(np.clip(int(os.environ.get("SCENE_SCAN_EVERY", "3")), 1, 12))
SCENE_MAP_HIRES_SAMPLES = int(np.clip(int(os.environ.get("SCENE_MAP_HIRES_SAMPLES", "3")), 1, 8))

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

viz_settings   = {'mode': 'fieldlines', 'color_mode': 'hybrid', 'intensity': 78, 'ghost_persistence': 18, 'scan_speed': 3}
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
    while True:
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

threading.Thread(target=detection_loop, daemon=True).start()

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
    """Data-body break-up: scan corruption, block displacement, hex residue."""
    arr = np.array(img.convert('RGB'), dtype=np.uint8)
    h, w = arr.shape[:2]
    s = intensity / 100.0
    people = _safe_people(people, w, h)

    arr = arr.copy()
    # Global low-amplitude data bending.
    step = max(7, int(26 - 14 * s))
    for row in range((frame_num * 3) % step, h, step):
        band_h = min(2 + int(7 * s), h - row)
        shift = int(math.sin(frame_num * 0.25 + row * 0.027) * 20 * s)
        if band_h > 0:
            arr[row:row+band_h] = np.roll(arr[row:row+band_h], shift, axis=1)
    noise_mask = np.random.random((h, w)) < (0.006 + 0.018 * s)
    noise_val = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
    arr[noise_mask] = noise_val[noise_mask]

    for (x1, y1, x2, y2, conf) in people:
        rh, rw = y2 - y1, x2 - x1
        if rh < 4 or rw < 4:
            continue
        region = arr[y1:y2, x1:x2].copy()
        # Block quantization increases toward the lower body.
        stripe_h = max(3, rh // 16)
        for stripe in range(0, rh, stripe_h):
            se = min(stripe + stripe_h, rh)
            prog = stripe / max(1, rh)
            bs = max(2, int(2 + prog * 28 * s))
            sr = region[stripe:se, :]
            sh, sw = sr.shape[:2]
            if sh and sw:
                small = Image.fromarray(sr).resize((max(1, sw // bs), max(1, sh // bs)), Image.NEAREST)
                region[stripe:se, :] = np.array(small.resize((sw, sh), Image.NEAREST))
        # Horizontal tear bands.
        for _ in range(max(3, int(9 * s))):
            yy = random.randint(0, max(0, rh - 1))
            bh = random.randint(1, max(1, min(6, rh - yy)))
            sh = random.randint(-max(1, int(rw * 0.18 * s)), max(1, int(rw * 0.18 * s)))
            region[yy:yy+bh] = np.roll(region[yy:yy+bh], sh, axis=1)
        # Particle columns falling out of subject.
        for _ in range(max(1, int(rw * s * 0.08))):
            sx = random.randint(0, rw - 1)
            sy = random.randint(0, max(0, rh - 1))
            max_len = max(1, min(38, rh - sy))
            ln = random.randint(1, max_len)
            for dy in range(ln):
                fade = 1.0 - dy / max(1, ln)
                region[sy + dy, sx] = [0, int(255 * fade), int(90 * fade)]
        arr[y1:y2, x1:x2] = region

    result = Image.fromarray(arr).convert('RGBA')
    layer = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    font = _font(8)
    for (x1, y1, x2, y2, conf) in people:
        alpha = int(155 * s)
        for row in range(y1, min(y2, h), 12):
            hex_line = ''.join(f'{random.randint(0,255):02x} ' for _ in range(5))
            draw.text((min(w-74, x2 + 4), row), hex_line, fill=(0, 210, 90, alpha), font=font)
        draw.line([(x1, y1), (x2, y2)], fill=(0, 255, 120, int(70*s)), width=1)
        draw.line([(x2, y1), (x1, y2)], fill=(0, 255, 120, int(45*s)), width=1)
    _mode_stamp(layer, "DISSOLUTION", "body converted to unstable data", frame_num, s, accent=(0, 225, 100))
    return Image.alpha_composite(result, layer).convert('RGB')


def fx_shield(img, people, frame_num, intensity):
    """Privacy field: shields, interference rings, body-centered jamming."""
    arr = np.array(img.convert('RGB'), dtype=np.uint8)
    h, w = arr.shape[:2]
    s = intensity / 100.0
    people = _safe_people(people, w, h)
    targets = people if people else [(w//2 - w//5, h//2 - h//4, w//2 + w//5, h//2 + h//4, 0.45)]

    # Subtle radial wave distortion around targets before drawing the field.
    xn, yn = _normalized_grids(h, w)
    dx = np.zeros((h, w), dtype=np.float32)
    dy = np.zeros((h, w), dtype=np.float32)
    phase = frame_num * 0.12
    for (x1, y1, x2, y2, conf) in targets:
        px = ((x1 + x2) / 2 / w) * 2 - 1
        py = ((y1 + y2) / 2 / h) * 2 - 1
        rr = np.sqrt((xn - px)**2 + (yn - py)**2) + 1e-5
        wave = np.sin(rr * 42 - phase * 4) * np.exp(-rr * 2.8) * s * conf
        dx += (xn - px) / rr * wave * w * 0.018
        dy += (yn - py) / rr * wave * h * 0.018
    warped = _warp_image(arr, dx, dy)
    overlay = Image.fromarray(_film_grain(warped, int(4*s))).convert('RGBA')

    layer = Image.new('RGBA', overlay.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    for ti, (x1, y1, x2, y2, conf) in enumerate(targets):
        ccx, ccy = (x1 + x2) // 2, (y1 + y2) // 2
        body_r = max(20, max(x2 - x1, y2 - y1) // 2)
        for ring in range(int(8 + 8*s)):
            r = body_r + ring * max(5, int(10 * s))
            rp = r + int(6 * math.sin(phase * 1.7 + ring * 0.58))
            hue = (0.40 + ring * 0.025 + phase * 0.02 + ti * 0.12) % 1.0
            rgb = colorsys.hsv_to_rgb(hue, 0.78, 1.0)
            alpha = int(max(14, 190 - ring * 14) * s * (0.7 + 0.3 * conf))
            pts = []
            for deg in range(0, 361, 5):
                a = math.radians(deg)
                pulse = 1.0 + 0.075 * math.sin(a * 3 + phase * 2) + 0.04 * math.cos(a * 7 - phase)
                pts.append((int(ccx + math.cos(a) * rp * pulse), int(ccy + math.sin(a) * rp * pulse * 0.76)))
            draw.line(pts, fill=(int(rgb[0]*255), int(rgb[1]*255), int(rgb[2]*255), alpha), width=2 if ring < 3 else 1)
        for ray in range(20):
            a = ray / 20 * 2 * math.pi + phase * 0.45
            rl = body_r * (1.65 + 0.34 * s)
            rx, ry = int(ccx + math.cos(a) * rl), int(ccy + math.sin(a) * rl * 0.76)
            alpha = int(65 * s * (0.5 + 0.5 * math.sin(phase * 2 + ray)))
            draw.line([(ccx, ccy), (rx, ry)], fill=(0, 255, 210, alpha), width=1)
        gr = max(8, int(body_r * 0.22))
        draw.ellipse([(ccx-gr, ccy-gr), (ccx+gr, ccy+gr)], outline=(0,255,190,int(185*s)), width=2)
    _mode_stamp(layer, "FIELD SHIELD", "privacy aura / signal jamming", frame_num, s, accent=(0,255,200))
    return Image.alpha_composite(overlay, layer).convert('RGB')


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
#  RESONANCE CAMERA — LIVE ENVIRONMENTAL VISUALIZATIONS
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _env_values():
    snap = get_environment_snapshot()
    g = snap.get('geomag', {})
    p = snap.get('particles', {})
    r = snap.get('radiation', {})
    c = snap.get('cosmic', {})
    ga = float(np.clip(g.get('activity') or 0.0, 0.0, 1.0))
    # Particle intensity combines three physically different scales without hiding
    # their provenance: local/regional RadNet, regional NMDB, and spaceborne GOES.
    pa = float(np.clip(max(float(p.get('activity') or 0.0),
                           float(c.get('activity') or 0.0)*0.85,
                           float(r.get('activity') or 0.0)*0.70), 0.0, 1.0))
    return snap, g, p, ga, pa


def _env_palette(ga, pa):
    """Cool magnetic cyan/violet + warm particle orange/red."""
    mag = (65, int(175 + 65*ga), 255)
    particle = (255, int(80 + 120*(1-pa)), 45)
    secondary = (190, 80, 255)
    return mag, particle, secondary


def _draw_data_stamp(layer, title, snap, ga, pa):
    draw = ImageDraw.Draw(layer)
    w, h = layer.size
    g, p = snap.get('geomag', {}), snap.get('particles', {})
    r, c = snap.get('radiation', {}), snap.get('cosmic', {})
    station = g.get('station') or '--'
    rad = r.get('station') or '--'
    cosmic = c.get('station') or '--'
    p10 = p.get('proton_10') or 0.0
    txt = f"{title} / MAG {station} / RAD {rad} / CR {cosmic} / ≥10MeV {p10:.3g}"
    font = _font(9)
    tw = min(w-20, max(210, int(len(txt)*5.7)+18))
    draw.rounded_rectangle([(10,10),(tw,31)], radius=6, fill=(0,0,0,145), outline=(120,160,255,55), width=1)
    draw.text((17,16), txt[:92], font=font, fill=(220,230,235,205))


def fx_fieldlines(img, people, frame_num, intensity):
    """Field-line visualization driven by nearest USGS X/Y/Z/F data.

    This is the comparatively representational mode: field orientation uses the
    measured horizontal vector/declination, while short-term station variation
    changes curvature and motion. It is still a visualization, not a literal
    local field map at the camera.
    """
    base = img.convert('RGBA')
    w, h = base.size
    s = intensity / 100.0
    snap, g, p, ga, pa = _env_values()
    mag_col, particle_col, violet = _env_palette(ga, pa)
    layer = Image.new('RGBA', base.size, (0,0,0,0))
    draw = ImageDraw.Draw(layer)

    decl = math.radians(float(g.get('declination_deg') or 0.0))
    incl = math.radians(float(g.get('inclination_deg') or 60.0))
    phase = frame_num * (0.006 + 0.022*ga)
    cx, cy = w*0.50, h*0.52
    span = max(w, h) * 0.62
    line_count = int(10 + 16*s)

    # Dipole-like arcs projected into the image plane and rotated by the measured
    # horizontal magnetic direction. Curvature responds to inclination.
    for i in range(-line_count//2, line_count//2 + 1):
        offset = i / max(1, line_count//2)
        pts = []
        for j in range(46):
            t = -1.0 + 2.0*j/45.0
            x = t * span
            bend = (0.24 + 0.24*abs(math.sin(incl))) * span
            y = offset * (0.22*span) + bend * (t*t - 0.55)
            y += math.sin(t*5.0 + phase + i*0.45) * (2.0 + 10.0*ga) * s
            xr = x*math.cos(decl) - y*math.sin(decl)
            yr = x*math.sin(decl) + y*math.cos(decl)
            pts.append((int(cx+xr), int(cy+yr)))
        alpha = int((32 + 105*(1-abs(offset))) * s)
        draw.line(pts, fill=(*mag_col, alpha), width=1 if w < 1200 else 2)

    # Measured vector axis.
    vx = math.cos(decl) * min(w,h)*0.18
    vy = math.sin(decl) * min(w,h)*0.18
    draw.line([(cx-vx,cy-vy),(cx+vx,cy+vy)], fill=(*mag_col,int(180*s)), width=max(1,w//500))
    draw.ellipse([(cx-3,cy-3),(cx+3,cy+3)], fill=(*particle_col,int(210*s)))
    _draw_data_stamp(layer, 'FIELD / VECTOR', snap, ga, pa)
    return Image.alpha_composite(base, layer).convert('RGB')


def fx_particle_rain(img, people, frame_num, intensity):
    """GOES proton/electron flux rendered as incoming particle tracks."""
    base = img.convert('RGBA')
    w, h = base.size
    s = intensity / 100.0
    snap, g, p, ga, pa = _env_values()
    mag_col, particle_col, violet = _env_palette(ga, pa)
    layer = Image.new('RGBA', base.size, (0,0,0,0))
    draw = ImageDraw.Draw(layer)

    # Keep particle positions deterministic per index: no per-frame RNG churn.
    n = int(26 + s*54 + pa*90)
    speed = 1.4 + 5.0*pa
    angle = -0.15 + 0.30*math.sin(frame_num*0.002 + ga*2.0)
    for i in range(n):
        seed = (i * 2654435761) & 0xffffffff
        x0 = ((seed % 10000) / 10000.0) * w
        yphase = (((seed >> 8) % 10000) / 10000.0) * (h + 120)
        y = (yphase + frame_num*speed*(1 + (i%7)*0.035)) % (h + 120) - 60
        length = 5 + (i % 11) + int(18*pa)
        dx = math.sin(angle) * length
        dy = math.cos(angle) * length
        # Rare brighter, higher-energy events.
        high = (i % max(3, int(14 - 8*pa))) == 0
        col = violet if high else particle_col
        a = int((80 + (110 if high else 30))*s)
        draw.line([(x0-dx, y-dy), (x0+dx, y+dy)], fill=(*col,a), width=2 if high else 1)
        if high:
            r = 1 + int(2*pa)
            draw.ellipse([(x0-r,y-r),(x0+r,y+r)], fill=(*col,min(255,a+30)))

    # Energy-band traces at bottom: 10 / 50 / 100 MeV.
    vals = [float(p.get('proton_10') or 0), float(p.get('proton_50') or 0), float(p.get('proton_100') or 0)]
    for k, val in enumerate(vals):
        yy = h - 18 - k*7
        frac = float(np.clip((math.log10(max(val,1e-5))+5)/7,0,1))
        draw.line([(12,yy),(12+int(frac*min(180,w*0.35)),yy)], fill=(*particle_col,int((100+k*35)*s)), width=2)
    _draw_data_stamp(layer, 'PARTICLE / ARRIVAL', snap, ga, pa)
    return Image.alpha_composite(base, layer).convert('RGB')


def fx_magnetosphere(img, people, frame_num, intensity):
    """Schematic Earth-field shell with GOES particles interacting at the edge."""
    base = img.convert('RGBA')
    w, h = base.size
    s = intensity / 100.0
    snap, g, p, ga, pa = _env_values()
    mag_col, particle_col, violet = _env_palette(ga, pa)
    layer = Image.new('RGBA', base.size, (0,0,0,0))
    draw = ImageDraw.Draw(layer)
    cx, cy = w*0.50, h*0.55
    R = min(w,h)*0.16
    pulse = 1.0 + 0.035*math.sin(frame_num*0.07)

    # Earth / camera locus.
    draw.ellipse([(cx-R,cy-R),(cx+R,cy+R)], outline=(*mag_col,int(150*s)), width=max(1,w//480))
    for shell in range(1,6):
        rx = R*(1.0+shell*0.52)*pulse
        ry = R*(1.0+shell*0.24)
        skew = (shell**1.15) * R * (0.14 + 0.18*ga)
        box = [(cx-rx-skew, cy-ry),(cx+rx-skew*0.15,cy+ry)]
        draw.arc(box, 200, 520, fill=(*mag_col,int((34+18*(6-shell))*s)), width=1 if w<1200 else 2)

    # Solar-particle ingress from one side, density from measured GOES activity.
    n = int(8 + 38*pa*s)
    for i in range(n):
        yy = cy - R*1.8 + (i/max(1,n-1))*R*3.6
        jitter = math.sin(i*1.73 + frame_num*0.08)*R*0.14
        x1 = w + 15
        x2 = cx + R*2.6 + jitter
        draw.line([(x1,yy),(x2,yy+jitter*0.25)], fill=(*particle_col,int(105*s)), width=1)
        draw.ellipse([(x2-2,yy+jitter*0.25-2),(x2+2,yy+jitter*0.25+2)], fill=(*particle_col,int(190*s)))

    _draw_data_stamp(layer, 'MAGNETOSPHERE / SCHEMATIC', snap, ga, pa)
    return Image.alpha_composite(base, layer).convert('RGB')


def fx_signal_veil(img, people, frame_num, intensity):
    """Abstract: translucent signal membranes driven by magnetic + particle data."""
    base = img.convert('RGBA')
    w, h = base.size
    s = intensity / 100.0
    snap, g, p, ga, pa = _env_values()
    mag_col, particle_col, violet = _env_palette(ga, pa)
    layer = Image.new('RGBA', base.size, (0,0,0,0))
    draw = ImageDraw.Draw(layer)
    bands = int(5 + 7*s)
    for b in range(bands):
        pts=[]
        phase = frame_num*(0.006+0.016*pa)+b*0.8
        amp = (12 + 55*ga + 35*pa) * s
        y0 = (b+1)*h/(bands+1)
        for x in range(-20,w+21,max(12,w//42)):
            y = y0 + math.sin(x*0.012 + phase)*amp + math.sin(x*0.003-phase*0.7)*amp*0.45
            pts.append((x,int(y)))
        col = mag_col if b%2==0 else violet
        draw.line(pts, fill=(*col,int((40+50*(b%3))*s)), width=max(1,w//700))
        if b%3==0:
            shifted=[(x,y+int(4+18*pa)) for x,y in pts]
            draw.line(shifted, fill=(*particle_col,int(40*s)), width=1)
    # Sparse discontinuities / data scars.
    for i in range(int(8+22*s)):
        x = int(((i*0.6180339 + frame_num*0.0015) % 1.0)*w)
        y = int(((i*0.381966 + frame_num*0.0008) % 1.0)*h)
        ln = int(8 + (ga+pa)*45 + (i%5)*4)
        draw.line([(x,y),(min(w,x+ln),y)], fill=(*particle_col,int(70*s)), width=1)
    _draw_data_stamp(layer, 'SIGNAL / VEIL', snap, ga, pa)
    return Image.alpha_composite(base, layer).convert('RGB')


def fx_resonance_map(img, people, frame_num, intensity):
    """Abstract: broken topographic rings, interference, erasure and memory."""
    base = img.convert('RGBA')
    w,h = base.size
    s=intensity/100.0
    snap,g,p,ga,pa = _env_values()
    mag_col,particle_col,violet = _env_palette(ga,pa)
    layer=Image.new('RGBA',base.size,(0,0,0,0)); draw=ImageDraw.Draw(layer)
    cx = w*(0.5 + 0.08*math.sin(frame_num*0.004 + ga*2.0))
    cy = h*(0.5 + 0.06*math.cos(frame_num*0.003 + pa*3.0))
    rings=int(8+18*s)
    for i in range(rings):
        r=(i+1)*min(w,h)/(2.2*rings)
        wobble=1+0.06*math.sin(frame_num*0.02+i*1.7)*(ga+pa)
        rx=r*wobble; ry=r*(0.72+0.12*math.sin(i+ga*4))
        start=(i*31 + frame_num*(0.3+pa))%360
        extent=170 + 130*math.sin(i*0.7+ga*2.0)
        col=(mag_col if i%3 else particle_col)
        draw.arc([(cx-rx,cy-ry),(cx+rx,cy+ry)],start,start+extent,fill=(*col,int((28+75*(1-i/max(1,rings)))*s)),width=1 if w<1100 else 2)
    # Small rectangular absences evoke cut/erased photographic fragments.
    for i in range(int(4+9*s)):
        x=int(((i*0.2718+frame_num*0.0006)%1)*w); y=int(((i*0.4142)%1)*h)
        ww=int(8+36*((i%5)/4)); hh=max(1,int(1+5*pa))
        draw.rectangle([(x,y),(min(w,x+ww),min(h,y+hh))],fill=(0,0,0,int(100*s)))
        draw.line([(x,y),(min(w,x+ww),y)],fill=(*violet,int(90*s)),width=1)
    _draw_data_stamp(layer,'RESONANCE / TOPOGRAPHY',snap,ga,pa)
    return Image.alpha_composite(base,layer).convert('RGB')


def fx_data_bloom(img, people, frame_num, intensity):
    """Abstract: environmental values bloom around tracked bodies or frame center."""
    base=img.convert('RGBA'); w,h=base.size; s=intensity/100.0
    snap,g,p,ga,pa=_env_values(); mag_col,particle_col,violet=_env_palette(ga,pa)
    layer=Image.new('RGBA',base.size,(0,0,0,0)); draw=ImageDraw.Draw(layer)
    targets=_safe_people(people,w,h)
    centers=[((x1+x2)//2,(y1+y2)//2,max(20,(x2-x1+y2-y1)//4)) for x1,y1,x2,y2,_ in targets]
    if not centers: centers=[(w//2,h//2,int(min(w,h)*0.18))]
    for ti,(cx,cy,rr) in enumerate(centers[:5]):
        petals=int(12+20*(ga+pa)*0.5)
        for i in range(petals):
            a=(i/petals)*math.tau + frame_num*(0.003+0.01*pa)
            length=rr*(0.7+1.0*ga+0.45*math.sin(i*2.1+frame_num*0.02))
            x2=cx+math.cos(a)*length; y2=cy+math.sin(a)*length
            col=particle_col if i%4==0 else mag_col
            draw.line([(cx,cy),(x2,y2)],fill=(*col,int((25+85*(i%3==0))*s)),width=1)
            if i%5==0:
                r=2+int(3*pa); draw.ellipse([(x2-r,y2-r),(x2+r,y2+r)],fill=(*violet,int(120*s)))
    _draw_data_stamp(layer,'DATA / BLOOM',snap,ga,pa)
    return Image.alpha_composite(base,layer).convert('RGB')

def fx_local_radiation(img, people, frame_num, intensity):
    """Local/regional EPA RadNet measurement rendered as a breathing interference field."""
    base=img.convert('RGBA'); w,h=base.size; s=intensity/100.0
    snap,g,p,ga,pa=_env_values(); r=snap.get('radiation',{}); ra=float(np.clip(r.get('activity') or 0.0,0,1))
    mag_col,particle_col,violet=_env_palette(ga,pa)
    layer=Image.new('RGBA',base.size,(0,0,0,0)); draw=ImageDraw.Draw(layer)
    centers=_safe_people(people,w,h)
    loci=[((x1+x2)//2,(y1+y2)//2) for x1,y1,x2,y2,_ in centers[:4]] or [(w//2,h//2)]
    for li,(cx,cy) in enumerate(loci):
        rings=int(8+18*s)
        for i in range(rings):
            rr=(i+1)*min(w,h)*0.018*(1+0.8*ra)
            wob=1+0.09*math.sin(frame_num*0.035+i*0.9+li)
            col=particle_col if i%3==0 else violet
            a=int((22+90*(1-i/max(1,rings)))*s*(0.55+0.45*ra))
            draw.ellipse([(cx-rr*wob,cy-rr),(cx+rr*wob,cy+rr)],outline=(*col,a),width=1)
        # sparse measurement flecks; deterministic and cheap
        for i in range(int(12+60*ra*s)):
            ang=(i*2.399963+frame_num*0.008)%math.tau; rad=min(w,h)*(0.04+0.28*((i*0.618)%1))
            x=cx+math.cos(ang)*rad; y=cy+math.sin(ang)*rad
            draw.point((int(x),int(y)),fill=(*particle_col,int(100*s)))
    _draw_data_stamp(layer,'RADNET / LOCAL FIELD',snap,ga,pa)
    return Image.alpha_composite(base,layer).convert('RGB')


def fx_cosmic_cascade(img, people, frame_num, intensity):
    """Regional NMDB neutron counts + GOES flux rendered as atmospheric cascades."""
    base=img.convert('RGBA'); w,h=base.size; s=intensity/100.0
    snap,g,p,ga,pa=_env_values(); c=snap.get('cosmic',{}); ca=float(np.clip(c.get('activity') or 0.0,0,1))
    mag_col,particle_col,violet=_env_palette(ga,pa)
    layer=Image.new('RGBA',base.size,(0,0,0,0)); draw=ImageDraw.Draw(layer)
    n=int(10+34*s+55*max(ca,float(p.get('activity') or 0.0)))
    for i in range(n):
        seed=(i*1103515245+12345)&0x7fffffff
        x=((seed%10000)/10000.0)*w
        phase=((seed>>7)%10000)/10000.0
        y=((phase*h)+frame_num*(1.2+4.0*pa)*(1+(i%5)*0.08))%(h+80)-40
        length=12+int(28*(0.35+ca))+i%13
        draw.line([(x,y-length),(x,y)],fill=(*particle_col,int(80+90*s)),width=1)
        if i%3==0:
            branches=2+(i%3)
            for b in range(branches):
                ang=(-0.8+1.6*b/max(1,branches-1))+0.2*math.sin(frame_num*0.02+i)
                L=length*(0.5+0.35*((b+1)/branches))
                draw.line([(x,y),(x+math.sin(ang)*L,y+math.cos(ang)*L)],fill=(*violet,int(65+80*s)),width=1)
    _draw_data_stamp(layer,'COSMIC / CASCADE',snap,ga,pa)
    return Image.alpha_composite(base,layer).convert('RGB')


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  HUD
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def draw_hud(img, mode_label, people_count, frame_num):
    font = _font(11)
    with tap_regions_lock:
        tap_count = len(tap_regions)
    with _geiger_lock:
        cpm  = geiger_data['cpm']
        usvh = geiger_data['usvh']
        g_status = geiger_data.get('status', 'unknown')
    with _detection_lock:
        total_seen = int(detection_data.get('total_seen', 0))
        detect_status = str(detection_data.get('status', 'waiting')).upper()[:22]
    with _background_lock:
        bg_frames = _background_frames
        bg_empty_frames = _background_empty_frames
        bg_ready = _background_model is not None and bg_empty_frames >= ERASE_BG_READY_EMPTY_FRAMES
    snap = get_environment_snapshot()
    env_g = snap.get('geomag', {})
    env_p = snap.get('particles', {})
    hud_lines = [
        (f"MODE: {mode_label}",                        (0,255,100)),
        (f"DETECTED: {people_count}",                  (255,80,60)),
        (f"TOTAL: {total_seen}",                       (180,180,180)),
        (f"DETECT: {detect_status}",                   (180,180,180)),
    ]
    if mode_label == 'ERASED':
        hud_lines.append((f"BG MODEL: {'READY' if bg_ready else 'WARMING'} ({bg_empty_frames}/{ERASE_BG_READY_EMPTY_FRAMES})", (0,220,120)))
    if tap_count > 0:
        hud_lines.append((f"TAP REGIONS: {tap_count}", (0,200,255)))
    st = env_g.get('station') or '--'
    dist = env_g.get('distance_km')
    dist_txt = f"{dist:.0f}km" if isinstance(dist,(int,float)) else '--'
    field_f = env_g.get('f')
    field_txt = f"{field_f:.1f}nT" if isinstance(field_f,(int,float)) else '--'
    p10 = float(env_p.get('proton_10') or 0.0)
    env_r = snap.get('radiation', {}); env_c = snap.get('cosmic', {})
    rad_station = env_r.get('station') or '--'; rad_gamma = env_r.get('gamma_mean')
    rad_txt = f"{rad_gamma:.2f}" if isinstance(rad_gamma,(int,float)) else '--'
    cr_station = env_c.get('station') or '--'; cr_rate = env_c.get('count_rate')
    cr_txt = f"{cr_rate:.2f}" if isinstance(cr_rate,(int,float)) else '--'
    hud_lines.append((f"USGS MAG: {st} {dist_txt}  F {field_txt}", (80,205,255)))
    hud_lines.append((f"EPA RADNET: {rad_station}  GAMMA {rad_txt}", (255,90,180)))
    hud_lines.append((f"NMDB COSMIC: {cr_station}  RATE {cr_txt}", (180,120,255)))
    hud_lines.append((f"NOAA GOES: ≥10MeV {p10:.3g} pfu  {snap.get('status','--').upper()}", (255,125,55)))
    cpm_color = (255,60,60) if g_status in ('noisy/stuck', 'offline') or cpm > 50 else (0,180,100)
    hud_lines.append((f"LOCAL TEST: {cpm:.1f} CPM  {usvh:.4f}µSv/h  {g_status.upper()}", cpm_color))
    # The magnetometer is polled outside the 30 fps render path.
    with _mag_lock:
        heading = _mag_last_heading
    if heading is not None:
        dirs = ['N','NE','E','SE','S','SW','W','NW']
        card = dirs[int((heading+22.5)/45)%8]
        hud_lines.append((f"HEADING: {heading}° {card}", (200,180,255)))
    else:
        hud_lines.append(("HEADING: SENSOR OFFLINE", (200,120,255)))
    box_h = 16 + len(hud_lines)*16
    box   = Image.new('RGBA', img.size, (0,0,0,0))
    bd    = ImageDraw.Draw(box)
    box_w = min(img.size[0] - 8, 340)
    bd.rounded_rectangle([(8,8),(box_w,box_h)], radius=8, fill=(10,10,15,200))
    bd.rounded_rectangle([(8,8),(box_w,box_h)], radius=8, outline=(0,180,80,60), width=1)
    img  = Image.alpha_composite(img.convert('RGBA'), box).convert('RGB')
    draw = ImageDraw.Draw(img)
    for i, (text, color) in enumerate(hud_lines):
        draw.text((16, 14+i*16), text, fill=color, font=font)
    return img

# ─── Still-camera rendering helpers ───────────────────────────────────────────
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
    """Prepare the base image for the selected global color treatment.

    hybrid (default): crisp B&W photograph + colored visualization overlays
    color:            color photograph + colored visualization overlays
    mono:             B&W photograph; final visualization is also desaturated
    """
    rgb = np.ascontiguousarray(frame[:, :, :3])
    rotated = cv2.rotate(rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)
    color_mode = str(viz_settings.get('color_mode', 'hybrid'))
    if color_mode == 'color':
        blur = cv2.GaussianBlur(rotated, (0, 0), 1.0)
        sharp = cv2.addWeighted(rotated, 1.28, blur, -0.28, 0)
        return Image.fromarray(np.clip(sharp, 0, 255).astype(np.uint8))
    gray = cv2.cvtColor(rotated, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (0, 0), 1.1)
    sharp = cv2.addWeighted(gray, 1.65, blur, -0.65, 0)
    return Image.fromarray(cv2.cvtColor(np.clip(sharp,0,255).astype(np.uint8), cv2.COLOR_GRAY2RGB))


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

    if   mode == 'fieldlines':    out = fx_fieldlines(img, people, _frame_count, intensity); label='FIELD LINES'
    elif mode == 'localradiation': out = fx_local_radiation(img, people, _frame_count, intensity); label='LOCAL RADIATION'
    elif mode == 'cosmiccascade':  out = fx_cosmic_cascade(img, people, _frame_count, intensity); label='COSMIC CASCADE'
    elif mode == 'particles':   out = fx_particle_rain(img, people, _frame_count, intensity); label='PARTICLE ARRIVAL'
    elif mode == 'magnetosphere': out = fx_magnetosphere(img, people, _frame_count, intensity); label='MAGNETOSPHERE'
    elif mode == 'signalveil':  out = fx_signal_veil(img, people, _frame_count, intensity); label='SIGNAL VEIL'
    elif mode == 'resonance':   out = fx_resonance_map(img, people, _frame_count, intensity); label='RESONANCE MAP'
    elif mode == 'databloom':   out = fx_data_bloom(img, people, _frame_count, intensity); label='DATA BLOOM'
    elif mode == 'panopticon':  out = fx_panopticon(img, people, _frame_count, intensity); label='PANOPTICON'
    elif mode == 'ghost':       out = fx_ghost(img, people, trails, _frame_count, intensity); label='GHOST'
    elif mode == 'dissolution': out = fx_dissolution(img, people, _frame_count, intensity); label='DISSOLUTION'
    elif mode == 'shield':      out = fx_shield(img, people, _frame_count, intensity); label='FIELD SHIELD'
    elif mode == 'erased':      out = fx_erased(img, people, _frame_count, intensity, source='adaptive'); label='ERASED'
    elif mode == 'mapped':      out = fx_erased(img, people, _frame_count, intensity, source='mapped'); label='MAPPED ERASE'
    elif mode == 'all':
        out = fx_ghost(img, people, trails, _frame_count, intensity * 0.5)
        out = fx_dissolution(out, people, _frame_count, intensity * 0.5)
        out = fx_shield(out, people, _frame_count, intensity * 0.6)
        out = fx_panopticon(out, people, _frame_count, intensity * 0.4)
        label = 'ALL LAYERS'
    elif mode == 'hyperspace':  out = fx_hyperspace(img, people, _frame_count, intensity); label='4D HYPERSPACE'
    elif mode == 'raw':         out = fx_raw(img); label='CAMERA'
    else:                       out = img; label='CAMERA'

    out = apply_tap_removals(out)
    if str(viz_settings.get('color_mode', 'hybrid')) == 'mono':
        out = ImageOps.grayscale(out).convert('RGB')
    if include_hud:
        out = draw_hud(out, label, len(people), _frame_count)
        if str(viz_settings.get('color_mode', 'hybrid')) == 'mono':
            out = ImageOps.grayscale(out).convert('RGB')
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

# ─── Shared phone-preview producer ─────────────────────────────────────────────
_stream_condition = threading.Condition()
_stream_jpeg = None
_stream_sequence = 0
_stream_started = False
_stream_start_lock = threading.Lock()
_stream_clients = 0
_stream_stats = {'fps': 0.0, 'frame_ms': 0.0, 'quality': PREVIEW_JPEG_QUALITY}


def stream_producer_loop():
    """Render/encode once, then share the same JPEG with every connected client."""
    global _frame_count, _latest_frame, _latest_frame_id
    global _stream_jpeg, _stream_sequence
    period = 1.0 / max(1.0, PREVIEW_FPS)
    next_deadline = time.monotonic()
    fps_ema = 0.0
    last_publish = None
    while True:
        started = time.monotonic()
        try:
            frame = _capture_stream_array('lores')
            with _frame_lock:
                _latest_frame = frame.copy()
                _latest_frame_id += 1
            _frame_count += 1
            out = render_processed_frame(frame, include_hud=True, learn_background=True)
            rgb = np.asarray(out.convert('RGB'))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, encoded = cv2.imencode(
                '.jpg', bgr,
                [cv2.IMWRITE_JPEG_QUALITY, PREVIEW_JPEG_QUALITY,
                 cv2.IMWRITE_JPEG_OPTIMIZE, 0]
            )
            if not ok:
                raise RuntimeError('preview JPEG encoding failed')
            jpeg = encoded.tobytes()
            publish_now = time.monotonic()
            elapsed = max(1e-4, publish_now - started)
            if last_publish is None:
                instant_fps = PREVIEW_FPS
            else:
                instant_fps = 1.0 / max(1e-4, publish_now - last_publish)
            last_publish = publish_now
            fps_ema = instant_fps if fps_ema == 0 else fps_ema * 0.88 + instant_fps * 0.12
            _stream_stats.update({'fps': round(fps_ema, 1),
                                  'frame_ms': round(elapsed * 1000.0, 1)})
            with _stream_condition:
                _stream_jpeg = jpeg
                _stream_sequence += 1
                _stream_condition.notify_all()
        except Exception as e:
            print(f"Frame error: {e}")
            time.sleep(0.10)

        next_deadline += period
        now = time.monotonic()
        if next_deadline < now - period:
            next_deadline = now
        if next_deadline > now:
            time.sleep(next_deadline - now)


def ensure_stream_producer():
    global _stream_started
    with _stream_start_lock:
        if not _stream_started:
            threading.Thread(target=stream_producer_loop, daemon=True, name='preview-producer').start()
            _stream_started = True


def generate_frames():
    global _stream_clients
    ensure_stream_producer()
    last_sequence = -1
    with _stream_condition:
        _stream_clients += 1
    try:
        while True:
            with _stream_condition:
                _stream_condition.wait_for(
                    lambda: _stream_jpeg is not None and _stream_sequence != last_sequence,
                    timeout=2.0,
                )
                if _stream_jpeg is None:
                    continue
                jpeg = _stream_jpeg
                last_sequence = _stream_sequence
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                   b'Cache-Control: no-store\r\n\r\n' + jpeg + b'\r\n')
    finally:
        with _stream_condition:
            _stream_clients = max(0, _stream_clients - 1)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  HTML
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

HTML = '''
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Resonance Camera — Environmental Imaging</title>
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
    <div class="brand"><div class="mark">◉</div><div><h1>Resonance Camera</h1><small>reverse surveillance / 4D vision engine</small></div></div>
    <div class="live">LIVE STILL</div>
  </div>
  <div class="layout">
    <main class="stage" id="videoPanel">
      <img id="feed" src="{{ url_for('video_feed') }}" alt="Live camera feed">
      <canvas class="tap-overlay" id="tapCanvas"></canvas>
      <div class="stageHud">
        <div class="chip green" id="modeChip">FIELD LINES</div>
        <div class="chip" id="detectChip">DETECT: --</div>
        <div class="chip violet" id="bgChip">BG: WARMING</div>
      </div>
      <div class="modes" id="modes">
        <button class="mode active" data-mode="fieldlines" onclick="setMode('fieldlines')">Field Lines</button>
        <button class="mode" data-mode="localradiation" onclick="setMode('localradiation')">Local Radiation</button>
        <button class="mode" data-mode="cosmiccascade" onclick="setMode('cosmiccascade')">Cosmic Cascade</button>
        <button class="mode" data-mode="particles" onclick="setMode('particles')">Particle Arrival</button>
        <button class="mode" data-mode="magnetosphere" onclick="setMode('magnetosphere')">Magnetosphere</button>
        <button class="mode" data-mode="signalveil" onclick="setMode('signalveil')">Signal Veil</button>
        <button class="mode" data-mode="resonance" onclick="setMode('resonance')">Resonance Map</button>
        <button class="mode" data-mode="databloom" onclick="setMode('databloom')">Data Bloom</button>
        <button class="mode" data-mode="ghost" onclick="setMode('ghost')">Ghost</button>
        <button class="mode" data-mode="dissolution" onclick="setMode('dissolution')">Dissolution</button>
        <button class="mode" data-mode="shield" onclick="setMode('shield')">Shield</button>
        <button class="mode" data-mode="hyperspace" onclick="setMode('hyperspace')">4D Hyperspace</button>
        <button class="mode" data-mode="raw" onclick="setMode('raw')">Camera</button>
      </div>
    </main>
    <aside class="panel">
      <section class="card">
        <div class="section">Detection <span id="detStatus">SEARCHING</span></div>
        <div class="hero">
          <div class="metric red"><div class="v" id="det-count">0</div><div class="l">current subjects</div></div>
          <div class="metric"><div class="v" id="det-total">0</div><div class="l">total detections</div></div>
          <div class="metric cyan"><div class="v" id="geomag-field">--</div><div class="l">field nT</div></div>
          <div class="metric violet"><div class="v" id="particle-flux">--</div><div class="l">≥10 MeV pfu</div></div>
        </div>
      </section>
      <section class="card">
        <div class="section">Mode Concept <span id="modeMini">V1</span></div>
        <div class="desc" id="mode-desc"><strong>FIELD LINES</strong> — A measured USGS field vector becomes a moving, dipole-like projection across the image.</div>
      </section>
      <section class="card">
        <div class="section">Color Treatment <span id="colorModeLabel">B&W + COLOR</span></div>
        <div class="submodes">
          <button class="submode active color-choice" data-color="hybrid" onclick="setColorMode('hybrid')">B&W + Color</button>
          <button class="submode color-choice" data-color="color" onclick="setColorMode('color')">Full Color</button>
          <button class="submode color-choice" data-color="mono" onclick="setColorMode('mono')">Full B&W</button>
        </div>
        <div class="note"><b>Default:</b> the camera image stays black-and-white while environmental visualization layers remain in color.</div>
      </section>
      <section class="card" id="effectPanel">
        <div class="section">Effect Engine <span>LIVE</span></div>
        <div class="sliders">
          <div><div class="slider-label"><span>Intensity</span><b id="int-val">78%</b></div><input type="range" id="intensity" min="0" max="100" value="78" oninput="updateFx()"></div>
          <div><div class="slider-label"><span>Persistence</span><b id="gp-val">18</b></div><input type="range" id="ghost_persist" min="2" max="30" value="18" oninput="updateFx()"></div>
          <div><div class="slider-label"><span>Scan / Motion Speed</span><b id="ss-val">3</b></div><input type="range" id="scan_speed" min="1" max="8" value="3" oninput="updateFx()"></div>
        </div>
        <div class="note"><b>Adaptive Erase</b> learns the empty scene automatically. It now replaces people with clean-plate pixels instead of inpainting or blurring them.</div>
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
      <div class="note"><b>Low-latency phone preview / 2K saved stills. Environmental APIs refresh asynchronously and never block frame rendering.</b> Saved JPEGs use the 2560×1440 main stream (1440×2560 after portrait rotation) at high quality. Re-scan Mapped Erase after installing this version to create a native-resolution clean plate.</div>
      <div class="btns"><button class="btn primary" onclick="capture()">⊙ Capture 2K Still</button><button class="btn ghost" onclick="resetAll()">Reset Installation</button></div>
    </aside>
  </div>
  <div class="flash" id="flashOverlay"></div><div class="toast" id="toast"></div>
<script>
const descriptions={
  fieldlines:'<strong>FIELD LINES</strong> — Uses the nearest available USGS ground station vector and recent variation to draw a dipole-like field projection. Orientation follows measured X/Y direction; this is a data-driven visualization, not a literal field map at the camera.',
  localradiation:'<strong>LOCAL RADIATION</strong> — EPA RadNet gamma measurements from the nearest configured local/regional monitor become concentric interference fields around tracked bodies or the frame center.',
  cosmiccascade:'<strong>COSMIC CASCADE</strong> — Regional NMDB neutron-monitor counts and GOES particle flux become branching atmospheric shower traces: a schematic of particles arriving from space and generating secondary cascades.',
  particles:'<strong>PARTICLE ARRIVAL</strong> — NOAA GOES proton/electron flux becomes an animated rain of incoming tracks. Density and speed respond to current particle activity and energy channels.',
  magnetosphere:'<strong>MAGNETOSPHERE</strong> — A schematic field shell combines ground magnetic measurements with incoming GOES particles to evoke interception and deflection around Earth.',
  signalveil:'<strong>SIGNAL VEIL</strong> — Abstract membranes, tears, and discontinuities driven by magnetic variation and particle flux.',
  resonance:'<strong>RESONANCE MAP</strong> — Broken topographic rings and absences turn regional environmental measurements into a shifting memory-map.',
  databloom:'<strong>DATA BLOOM</strong> — Environmental measurements radiate from tracked bodies or the frame center as unstable diagrammatic growth.',
  panopticon:'<strong>PANOPTICON</strong> — Legacy institutional-gaze mode retained from the earlier camera build.',
  ghost:'<strong>GHOST</strong> — The body becomes an afterimage and the background leaks through it.',
  dissolution:'<strong>DISSOLUTION</strong> — Identity breaks into unstable data, displaced blocks, and residue.',
  shield:'<strong>FIELD SHIELD</strong> — Privacy aura rings, lens distortion, and signal-jamming interference.',
  erased:'<strong>ADAPTIVE ERASE</strong> — Motion-compensated clean-plate replacement.',
  mapped:'<strong>MAPPED ERASE</strong> — Uses a saved person-free scene map for replacement.',
  hyperspace:'<strong>4D HYPERSPACE</strong> — Live pixels rendered as animated W-axis cross-sections and projected 4D geometry.',
  all:'<strong>ALL LAYERS</strong> — Legacy dense composite mode.',
  raw:'<strong>CAMERA</strong> — Unadorned camera feed using the selected global color treatment.'
};
const labels={fieldlines:'FIELD LINES',localradiation:'LOCAL RADIATION',cosmiccascade:'COSMIC CASCADE',particles:'PARTICLE ARRIVAL',magnetosphere:'MAGNETOSPHERE',signalveil:'SIGNAL VEIL',resonance:'RESONANCE MAP',databloom:'DATA BLOOM',panopticon:'PANOPTICON',ghost:'GHOST',dissolution:'DISSOLUTION',shield:'SHIELD',erased:'ADAPTIVE ERASE',mapped:'MAPPED ERASE',hyperspace:'4D HYPERSPACE',all:'ALL LAYERS',raw:'CAMERA'};
let currentMode='fieldlines';let tapMode=false,tapStart=null,lastCaptureId=null,_pollT=0;const postTimers={};function postSoon(key,url,payload,delay=120){clearTimeout(postTimers[key]);postTimers[key]=setTimeout(()=>fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}).catch(()=>{}),delay)}
function clsFor(m){return m==='hyperspace'?'hyper':m==='raw'?'amber':''}
function setMode(m){currentMode=m;document.querySelectorAll('.mode').forEach(b=>{const a=b.dataset.mode===m;b.className='mode'+(a?' active '+clsFor(m):'')});document.getElementById('mode-desc').innerHTML=descriptions[m]||'';document.getElementById('modeChip').textContent=labels[m]||m.toUpperCase();document.getElementById('hyperPanel').style.display=m==='hyperspace'?'block':'none';document.getElementById('camPanel').style.display=m==='raw'?'block':'none';document.getElementById('erasePanel').style.display=(m==='erased'||m==='mapped')?'block':'none';document.getElementById('eraseSource').textContent=m==='mapped'?'MAPPED SCENE':'ADAPTIVE';document.getElementById('effectPanel').style.display=(m==='raw'||m==='hyperspace')?'none':'block';fetch('/viz_settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})});}
function setColorMode(m){document.querySelectorAll('.color-choice').forEach(b=>b.classList.toggle('active',b.dataset.color===m));document.getElementById('colorModeLabel').textContent=m==='hybrid'?'B&W + COLOR':m==='color'?'FULL COLOR':'FULL B&W';fetch('/viz_settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({color_mode:m})});}
function setHyperSub(sub){document.querySelectorAll('.submode').forEach(b=>b.classList.toggle('active',b.dataset.sub===sub));fetch('/hyperspace_settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({submode:sub})});}
function updateHyper(){const rs=parseFloat(document.getElementById('rotSpeed').value),wd=parseFloat(document.getElementById('wDepth').value);document.getElementById('rotSpeedVal').textContent=rs.toFixed(1)+'×';document.getElementById('wDepthVal').textContent=wd.toFixed(1);postSoon('hyper','/hyperspace_settings',{rot_speed:rs,w_depth:wd});}
function updateFx(){const iv=document.getElementById('intensity').value,gp=document.getElementById('ghost_persist').value,ss=document.getElementById('scan_speed').value;document.getElementById('int-val').textContent=iv+'%';document.getElementById('gp-val').textContent=gp;document.getElementById('ss-val').textContent=ss;postSoon('fx','/viz_settings',{intensity:parseInt(iv),ghost_persistence:parseInt(gp),scan_speed:parseInt(ss)});}
function updateCam(){const autoExp=document.getElementById('autoExp').checked,expTime=parseInt(document.getElementById('expTime').value),gain=parseFloat(document.getElementById('isoGain').value),ev=parseFloat(document.getElementById('evComp').value),bright=parseFloat(document.getElementById('camBright').value),contrast=parseFloat(document.getElementById('camContrast').value),sat=parseFloat(document.getElementById('camSat').value);document.getElementById('autoExpCtrl').style.display=autoExp?'block':'none';document.getElementById('manualExpCtrl').style.display=autoExp?'none':'block';document.getElementById('evVal').textContent=(ev>=0?'+':'')+ev.toFixed(1);document.getElementById('expTimeVal').textContent=expTime<1000?expTime+'µs':(expTime/1000).toFixed(0)+'ms';document.getElementById('isoVal').textContent='ISO '+Math.round(gain*100);document.getElementById('camBrightVal').textContent=(bright>=0?'+':'')+bright.toFixed(1);document.getElementById('camContrastVal').textContent=contrast.toFixed(1);document.getElementById('camSatVal').textContent=sat.toFixed(1);document.getElementById('camModeLabel').textContent=autoExp?'AUTO':'MANUAL';postSoon('cam','/cam_controls',{auto_exposure:autoExp,exposure_time:expTime,analogue_gain:gain,ev_compensation:ev,brightness:bright,contrast:contrast,saturation:sat},160);}
function toggleTap(){tapMode=!tapMode;document.getElementById('videoPanel').classList.toggle('tap-mode',tapMode);document.getElementById('tapBtn').classList.toggle('active',tapMode);document.getElementById('tapHint').style.display=tapMode?'block':'none';if(tapMode)toast('Drag on the image to remove an object');}
function coords(e){const img=document.getElementById('feed'),r=img.getBoundingClientRect();return{nx:Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),ny:Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))};}
const tapSurface=document.getElementById('tapCanvas');tapSurface.addEventListener('pointerdown',e=>{if(!tapMode)return;e.preventDefault();tapSurface.setPointerCapture(e.pointerId);tapStart=coords(e)});tapSurface.addEventListener('pointerup',endTap);tapSurface.addEventListener('pointercancel',()=>{tapStart=null});
function endTap(e){if(!tapMode||!tapStart)return;e.preventDefault();const c=coords(e);let nx1=Math.min(tapStart.nx,c.nx),ny1=Math.min(tapStart.ny,c.ny),nx2=Math.max(tapStart.nx,c.nx),ny2=Math.max(tapStart.ny,c.ny);const mn=.08;if(nx2-nx1<mn){const cx=(nx1+nx2)/2;nx1=Math.max(0,cx-mn/2);nx2=Math.min(1,cx+mn/2)}if(ny2-ny1<mn){const cy=(ny1+ny2)/2;ny1=Math.max(0,cy-mn/2);ny2=Math.min(1,cy+mn/2)}fetch('/tap_region',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x1:nx1,y1:ny1,x2:nx2,y2:ny2})}).then(r=>r.json()).then(d=>toast('Manual erasure region added: '+d.total));tapStart=null;}
function clearTap(){fetch('/tap_region/clear',{method:'POST'}).then(()=>toast('Manual erasure regions cleared'));}
function scanSceneMap(){setMode('mapped');fetch('/scene_map/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({frames:20})}).then(r=>r.json()).then(()=>toast('Scene mapping started — clear people; small handheld motion is okay')).catch(()=>toast('Could not start scene scan'));}
function clearSceneMap(){fetch('/scene_map/clear',{method:'POST'}).then(()=>toast('Saved scene map cleared')).catch(()=>toast('Could not clear scene map'));}
function flash(){const f=document.getElementById('flashOverlay');f.classList.add('show');setTimeout(()=>f.classList.remove('show'),70)}
function capture(){flash();fetch('/capture').then(r=>r.json()).then(d=>{if(d.capture_id!==undefined)lastCaptureId=d.capture_id;const dims=(d.width&&d.height)?' '+d.width+'×'+d.height:'';toast(d.success?'✓ 2K capture recorded'+dims+(d.display_name?': '+d.display_name:''):'✗ Capture failed')}).catch(()=>toast('✗ Capture failed'));}
function resetAll(){document.getElementById('intensity').value=78;document.getElementById('ghost_persist').value=18;document.getElementById('scan_speed').value=3;updateFx();setColorMode('hybrid');setMode('fieldlines');clearTap();}
function toast(msg){const el=document.getElementById('toast');el.textContent=msg;el.classList.add('show');setTimeout(()=>el.classList.remove('show'),2500)}
function poll(){fetch('/detection_data',{cache:'no-store'}).then(r=>r.json()).then(d=>{document.getElementById('det-count').textContent=d.count;document.getElementById('det-total').textContent=d.total_seen;document.getElementById('geomag-field').textContent=d.env_geomag_f!==null?Number(d.env_geomag_f).toFixed(0):'--';document.getElementById('particle-flux').textContent=d.env_proton_10!==null?Number(d.env_proton_10).toPrecision(3):'--';document.getElementById('detStatus').textContent=(d.detection_status||'unknown').toUpperCase().slice(0,18);document.getElementById('detectChip').textContent='DETECT: '+(d.detection_status||'UNKNOWN').toUpperCase().slice(0,14);const mappedReady=!!d.mapped_background_ready,adaptiveReady=!!d.background_ready,target=d.scene_scan_target||20,accepted=d.scene_scan_accepted||0,rejected=d.scene_scan_rejected||0,percent=Math.max(0,Math.min(100,d.scene_scan_percent||0)),phase=(d.scene_scan_phase||'idle').toUpperCase();document.getElementById('bgChip').textContent=currentMode==='mapped'?(mappedReady?'MAP: READY':'MAP: '+accepted+'/'+target):(adaptiveReady?'BG: READY':'BG: '+(d.background_empty_frames||0)+'/'+(d.background_required_frames||12));document.getElementById('mapStatus').textContent=(d.scene_scan_status||'not mapped').toUpperCase().slice(0,34);document.getElementById('mapProgress').textContent=accepted+'/'+target;document.getElementById('mapCounts').textContent=accepted+' / '+rejected;document.getElementById('mapPhase').textContent=phase;document.getElementById('mapAlignment').textContent=(d.scene_scan_alignment||d.alignment_status||'idle').toUpperCase().slice(0,30);document.getElementById('mapProgressBar').style.width=percent+'%';const busy=d.scene_scan_active||phase==='FINALIZING';document.getElementById('scanMapBtn').textContent=phase==='READY'?'Rebuild Clean Scene Map':busy?'Building Scene Map…':'Build Clean Scene Map';document.getElementById('scanMapBtn').disabled=!!busy;if(lastCaptureId===null){lastCaptureId=d.capture_id}else if(d.capture_id!==lastCaptureId){lastCaptureId=d.capture_id;flash();toast(d.capture_success?'✓ Capture recorded'+(d.capture_source==='hardware'?' from shutter':''):'✗ Capture failed')}_pollT+=.03;document.getElementById('wSlice').textContent=Math.sin(_pollT*.38).toFixed(3);document.getElementById('rotRad').textContent=(_pollT%(2*Math.PI)).toFixed(2)+' rad';document.getElementById('entities4d').textContent=d.count;}).catch(()=>{});}poll();setInterval(poll,1000);updateFx();updateCam();
</script>
</body>
</html>
'''

# ─── Routes ───────────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame', headers={'Cache-Control':'no-store, no-cache, must-revalidate','Pragma':'no-cache'})

@app.route('/detection_data')
def get_detection_data():
    with _geiger_lock:
        cpm  = geiger_data['cpm']
        usvh = geiger_data['usvh']
        g_status = geiger_data.get('status')
        g_line = geiger_data.get('line')
        g_rejected = geiger_data.get('rejected')
    heading = read_magnetometer()
    with _capture_lock:
        cap = dict(capture_state)
    env = get_environment_snapshot()
    eg = env.get('geomag', {})
    ep = env.get('particles', {})
    with _detection_lock:
        det = dict(detection_data)
        people = list(detection_data.get('people', []))
    with _background_lock:
        bg_frames = _background_frames
        bg_empty_frames = _background_empty_frames
        bg_ready = _background_model is not None and bg_empty_frames >= ERASE_BG_READY_EMPTY_FRAMES
    with _scene_map_lock:
        map_ready = _mapped_background is not None
        map_created = _mapped_created_at
        map_size = None if _mapped_background is None else [
            int(_mapped_background.shape[1]), int(_mapped_background.shape[0])
        ]
        scan_state = dict(_scene_scan_state)
    return jsonify({
        'count':      det.get('count', 0),
        'total_seen': det.get('total_seen', 0),
        'people':     [(x1,y1,x2,y2,c) for x1,y1,x2,y2,c in people],
        'tracks':     det.get('tracks', []),
        'detection_status': det.get('status', 'unknown'),
        'background_ready': bg_ready,
        'background_frames': bg_frames,
        'background_empty_frames': bg_empty_frames,
        'background_required_frames': ERASE_BG_READY_EMPTY_FRAMES,
        'mapped_background_ready': map_ready,
        'mapped_background_created_at': map_created,
        'mapped_background_size': map_size,
        'preview_size': list(PREVIEW_SIZE),
        'still_size': list(STILL_SIZE),
        'scene_scan_active': scan_state.get('active', False),
        'scene_scan_status': scan_state.get('status', 'not mapped'),
        'scene_scan_accepted': scan_state.get('accepted', 0),
        'scene_scan_rejected': scan_state.get('rejected', 0),
        'scene_scan_target': scan_state.get('target', SCENE_SCAN_TARGET),
        'scene_scan_percent': scan_state.get('percent', 0),
        'scene_scan_phase': scan_state.get('phase', 'idle'),
        'scene_scan_alignment': scan_state.get('alignment', 'idle'),
        'scene_scan_last_motion': scan_state.get('last_motion'),
        'alignment_status': _alignment_status.get('status', 'idle'),
        'alignment_details': dict(_alignment_status),
        'stream_fps': _stream_stats.get('fps', 0.0),
        'stream_frame_ms': _stream_stats.get('frame_ms', 0.0),
        'stream_clients': _stream_clients,
        'preview_fps_target': PREVIEW_FPS,
        'preview_jpeg_quality': PREVIEW_JPEG_QUALITY,
        'cpm':        cpm,
        'usvh':       usvh,
        'geiger_status': g_status,
        'geiger_line': g_line,
        'env_status': env.get('status'),
        'env_error': env.get('error'),
        'env_location': env.get('location'),
        'env_geomag_station': eg.get('station'),
        'env_geomag_distance_km': eg.get('distance_km'),
        'env_geomag_f': eg.get('f'),
        'env_geomag_declination': eg.get('declination_deg'),
        'env_geomag_variation_nt': eg.get('variation_nt'),
        'env_proton_10': ep.get('proton_10'),
        'env_proton_50': ep.get('proton_50'),
        'env_proton_100': ep.get('proton_100'),
        'env_electron': ep.get('electron'),
        'env_particle_time': ep.get('time'),
        'env_radnet_station': (env.get('radiation') or {}).get('station'),
        'env_radnet_gamma': (env.get('radiation') or {}).get('gamma_mean'),
        'env_radnet_distance_km': (env.get('radiation') or {}).get('distance_km'),
        'env_nmdb_station': (env.get('cosmic') or {}).get('station'),
        'env_nmdb_count_rate': (env.get('cosmic') or {}).get('count_rate'),
        'env_nmdb_distance_km': (env.get('cosmic') or {}).get('distance_km'),
        'geiger_rejected': g_rejected,
        'heading':    heading,
        'mag_status': MAG_STATUS,
        'capture_id': cap['id'],
        'capture_success': cap['success'],
        'capture_name': cap['display_name'],
        'capture_source': cap['source'],
        'capture_timestamp': cap['timestamp'],
        'capture_error': cap['error'],
    })

@app.route('/environment_data')
def get_environment_data_route():
    """Small diagnostic endpoint for verifying cached API values without touching render timing."""
    return jsonify(get_environment_snapshot())

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
    data = request.json or {}
    if 'mode' in data:
        mode = str(data['mode'])
        if mode in ('fieldlines','localradiation','cosmiccascade','particles','magnetosphere','signalveil','resonance','databloom','panopticon','ghost','dissolution','shield','erased','mapped','all','hyperspace','raw'):
            viz_settings['mode'] = mode
    if 'color_mode' in data:
        cm = str(data['color_mode']).lower()
        if cm in ('hybrid','color','mono'):
            viz_settings['color_mode'] = cm
    if 'intensity' in data:
        viz_settings['intensity'] = int(np.clip(int(data['intensity']), 0, 100))
    if 'ghost_persistence' in data:
        viz_settings['ghost_persistence'] = int(np.clip(int(data['ghost_persistence']), 2, 30))
    if 'scan_speed' in data:
        viz_settings['scan_speed'] = int(np.clip(int(data['scan_speed']), 1, 8))
    return jsonify({'success': True, 'settings': viz_settings})

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
        """Shared capture logic for the physical shutter button."""
        result = capture_current_frame(prefix='antisurv_btn', source='hardware')
        print("\n" + "="*50)
        if result.get('success'):
            print("  📸 SHUTTER BUTTON PRESSED")
            print("  ✓ Capture recorded")
            print(f"  File: {result.get('filename')}")
            print(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        else:
            print(f"  ✗ SHUTTER ERROR: {result.get('error')}")
        print("="*50 + "\n")

    def shutter_poll_loop():
        """Poll GPIO27 shutter button at 50ms intervals."""
        last = GPIO.HIGH
        while True:
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

    def power_poll_loop():
        """Poll the run/power switch.

        Default wiring is GPIO17/Pin 11 to GND.  The script waits for the switch
        to be held LOW for POWER_HOLD_SEC before acting, which prevents accidental
        shutdowns from switch bounce.
        """
        last = GPIO.HIGH
        low_since = None
        acted_while_low = False
        while True:
            try:
                state = GPIO.input(POWER_PIN)
                if state == GPIO.LOW:
                    if last == GPIO.HIGH:
                        low_since = time.monotonic()
                        acted_while_low = False
                    elif (not acted_while_low) and low_since is not None and (time.monotonic() - low_since) >= POWER_HOLD_SEC:
                        acted_while_low = True
                        if POWER_ACTION == "exit":
                            print(f"Power/run switch held LOW on GPIO{POWER_PIN} — exiting script...")
                            try: GPIO.cleanup()
                            except Exception: pass
                            os._exit(0)
                        else:
                            print(f"Power/run switch held LOW on GPIO{POWER_PIN} — shutting down Pi...")
                            try: GPIO.cleanup()
                            except Exception: pass
                            subprocess.call(['sudo', 'shutdown', '-h', 'now'])
                else:
                    low_since = None
                    acted_while_low = False
                last = state
            except Exception:
                pass
            time.sleep(0.05)

    threading.Thread(target=shutter_poll_loop, daemon=True).start()
    if POWER_PIN_ENABLED:
        threading.Thread(target=power_poll_loop, daemon=True).start()
        print(f"✓ GPIO polling active — shutter GPIO{BUTTON_PIN}, power/run GPIO{POWER_PIN} ({POWER_ACTION})")
    else:
        print(f"✓ GPIO polling active — shutter GPIO{BUTTON_PIN}; software power/run switch disabled")

# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("\n" + "="*60)
    print("  ANTI-SURVEILLANCE CAMERA v6 — Still Capture UI")
    print("="*60)
    print("  Access: http://raspberrypi.local:5000")
    print(f"  Shutter button:  GPIO{BUTTON_PIN} (default Pin 13)")
    print(f"  Geiger counter:  GPIO{GEIGER_PIN} (default Pin 16)")
    print("  Magnetometer:    I2C SDA/SCL breakout board — keep GPIO2/GPIO3 reserved for I2C")
    if HAS_GPIO and POWER_PIN_ENABLED:
        print(f"  Power/run switch: GPIO{POWER_PIN} (default Pin 11) — {POWER_ACTION}")
    else:
        print("  Power/run switch: disabled in script; use GPIO17 or systemd gpio-shutdown")
    print(f"  Capture folder:  {CAPTURE_DIR}")
    print(f"  Preview stream:  {PREVIEW_SIZE[0]}x{PREVIEW_SIZE[1]} @ {PREVIEW_FPS:.0f} fps, JPEG Q{PREVIEW_JPEG_QUALITY} (rotated {PREVIEW_SIZE[1]}x{PREVIEW_SIZE[0]})")
    print(f"  Saved stills:    {STILL_SIZE[0]}x{STILL_SIZE[1]} (rotated output {STILL_SIZE[1]}x{STILL_SIZE[0]}), JPEG Q{STILL_JPEG_QUALITY}")
    print("  Modes: panopticon, ghost, dissolution, shield,")
    print("         adaptive erase, mapped erase, all, 4D hyperspace, B&W camera")
    print("="*60 + "\n")
    _load_mapped_background()
    threading.Thread(target=get_model, daemon=True).start()
    ensure_stream_producer()
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    finally:
        if HAS_GPIO:
            GPIO.cleanup()
