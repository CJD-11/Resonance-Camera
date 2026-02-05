# Architecture — Technical Deep-Dive

Detailed technical documentation of the Magnetic Field Camera and Anti-Surveillance Camera processing pipelines.

---

## 1. Magnetic Field Camera

### Sensor Layer

The BMM150 Bosch magnetometer communicates over I2C (bus 1, address `0x13`). It is initialized by writing `0x01` to the power control register (`0x4B`) and `0x00` to the operation mode register (`0x4C`), setting it to continuous normal-power measurement mode.

Raw readings are 8 bytes starting at register `0x42`:

| Bytes | Axis | Resolution | Signed Range |
|-------|------|-----------|-------------|
| 0–1 | X | 13-bit | ±4096 → ×0.3 = µT |
| 2–3 | Y | 13-bit | ±4096 → ×0.3 = µT |
| 4–5 | Z | 15-bit | ±16384 → ×0.3 = µT |
| 6–7 | Hall | (unused) | — |

A background thread reads these values at 20 Hz (50ms interval) and updates a shared dictionary: `{x, y, z, magnitude}`.

### Distortion Engine — V2 (Standard)

The displacement map builder `_build_displacement_map()` takes image dimensions `(w, h)`, the magnetometer readings `(mx, my, mz)`, and a strength factor `(0–100)`.

**Coordinate grid:** NumPy `mgrid` creates pixel coordinate arrays. Each pixel's offset from center `(dx, dy)` and normalized radius `r_norm` (0 at center, 1 at corners) are precomputed.

**Layer 1 — Barrel/Pincushion:**
```
barrel_factor = 1.0 + k * r_norm²
```
Where `k = strength × 0.8 × tanh(magnitude / 80)`. Positive k pushes pixels outward (barrel), scaling proportional to field magnitude but saturating via tanh to prevent extreme values.

**Layer 2 — Swirl:**
```
θ = swirl_max × r_norm
```
Where `swirl_max = strength × 0.35 × nz` (normalized Z component). Applies a rotation matrix to the barrel-displaced coordinates. Pixels farther from center rotate more.

**Layer 3 — Directional Shift:**
```
shift_x = strength × 40 × nx
shift_y = strength × 40 × ny
```
Uniform push in the direction of the horizontal field vector, attenuated toward edges by `(1 - r_norm × 0.5)`.

The final `(map_x, map_y)` arrays are used as lookup indices into the source pixel array for nearest-neighbour remapping.

### Distortion Engine — V3 (Dramatic)

V3 adds four additional processing layers:

**4th-Order Barrel:** Adds `r_norm⁴` term for dramatically stronger edge warping:
```
barrel = 1.0 + k × r_norm² + 0.5 × strength × r_norm⁴
```

**Dipole Vortex:** Two counter-rotating swirl centers. The primary swirl is centered on the image; a secondary counter-swirl is offset in the direction of `(nx, ny)` with exponential falloff:
```
θ₁ = swirl × (1 - r_norm) × 2          (primary, stronger near center)
θ₂ = -swirl × 0.5 × exp(-r₂_norm × 3)  (secondary, offset, counter-rotating)
```

**Curl Ripple Waves:** Sinusoidal displacement perpendicular to the radial direction:
```
ripple = amplitude × sin(r_norm × frequency × π + phase)
tang_x = -dy / r × ripple    (tangential = curl-like)
tang_y =  dx / r × ripple
```
The `phase` variable increments each frame, creating animated wave propagation.

**Chromatic Aberration:** Each colour channel is displaced independently:
```
R channel: (ix + shift, iy + shift)
G channel: (ix, iy)              — sharpest
B channel: (ix - shift, iy - shift)
```
Where shift scales with `tanh(magnitude / 60)`.

### Overlay Renderer — V3

**Curl Streamlines (48 lines):** Each line starts at the center and steps outward, accumulating angle:
```
angle += curl_k × 0.3 + swirl_k × 0.15 + sin(step × 0.1 + phase) × 0.02
```
This produces spiralling curves whose curvature is physically driven by the magnetometer.

**Equipotential Rings:** Non-circular contour rings deformed by the field direction:
```
r_ring × (1 + 0.15 × s × cos(θ × 2 - atan2(my, mx))) × wobble
```

**Dipole Vector Field (28×28):** At each grid point, the dipole field formula is evaluated:
```
B_local = (3(m·r̂)r̂ - m) / (r × 0.01 + 1) + m × 0.03
```
Rendered as arrows with proper arrowheads.

### Camera Control Architecture

The Picamera2 `set_controls()` API maps to libcamera. A critical implementation detail: when switching to manual exposure, `AeEnable`, `ExposureTime`, and `AnalogueGain` must be sent in a single call. The script tracks manual values in a `_manual_controls` dictionary:

```python
_manual_controls = {"ExposureTime": 33000, "AnalogueGain": 1.0}
```

Any change to either value triggers:
```python
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": _manual_controls["ExposureTime"],
    "AnalogueGain": _manual_controls["AnalogueGain"],
})
```

The camera configuration also sets `FrameDurationLimits: (5000, 500000)` to allow the full range of shutter speeds.

Live metadata feedback is provided via `picam2.capture_metadata()` which returns the actual exposure time and gain the sensor is using, displayed in the UI's "Sensor Readout" section.

---

## 2. Anti-Surveillance Camera

### Detection Pipeline

YOLO runs in a dedicated background thread to avoid blocking the frame generator:

```
Main Thread                    Detection Thread
───────────                    ────────────────
capture_array()  ──copy──▶    YOLOv8n inference
      │                        (classes=[0], imgsz=320)
      │                              │
      ▼                              ▼
apply visual FX ◀── read ── detection_data dict
      │                      {people: [(x1,y1,x2,y2,conf)...]}
      ▼                      {ghost_trails: [frame_n, frame_n-1...]}
JPEG encode
      │
      ▼
MJPEG stream
```

The detection thread runs at approximately 6–7 Hz on a Pi 4. The frame generator runs independently, applying effects using the latest available detection data. This means visual effects update at frame rate even if detection is slightly behind.

### Ghost Trail System

Each frame's detections are appended to a circular buffer (`ghost_trails`). The persistence slider controls buffer length (2–30 frames). Older entries are rendered with increasing transparency and a green colour tint, creating the afterimage decay effect.

### Inpainting (Erased Mode)

OpenCV's TELEA inpainting algorithm fills the masked region by propagating pixel values from the boundary inward:

1. A binary mask is created from detection bounding boxes (with 15px padding)
2. `cv2.inpaint(bgr, mask, radius, cv2.INPAINT_TELEA)` fills the masked area
3. The inpaint radius scales with the intensity slider

The green outlined rectangle is then drawn on top of the inpainted result, marking where the person was.

### Visual Effects — Processing Order

| Mode | Steps |
|------|-------|
| Panopticon | Draw grid → scan line → person tracking boxes → biometric lines → status bar |
| Ghost | Desaturate + brighten person regions → blend ghost trails → add "NOT FOUND" text |
| Dissolution | Progressive pixelation → noise injection → data rain → hex overlay |
| Shield | Privacy aura rings → interference rays → shield glyph → screen-wide static noise |
| Erased | Build mask → inpaint → draw green rectangle + corners + label + scan line |
| All Layers | Ghost(50%) → Dissolution(50%) → Shield(60%) → Panopticon(40%) |

---

## 3. Flask Server Architecture

All scripts follow the same pattern:

```python
# MJPEG streaming via generator function
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
```

The `generate_frames()` generator yields JPEG frames continuously. The browser's `<img>` tag with the MJPEG content type handles display automatically — no JavaScript video player needed.

Settings are stored in shared dictionaries (`viz_settings`, `detection_data`) and modified via POST endpoints. Thread safety is adequate for single-writer patterns (the main thread or a single background thread writes; the frame generator reads).

### Inline HTML Template

The entire UI (HTML + CSS + JavaScript) is embedded as a Python string (`HTML = '''...'''`) and served via `render_template_string()`. This keeps deployment to a single file — no static assets directory needed. The UI polls `/mag_data` or `/detection_data` every 200–300ms for live sidebar updates.

---

*Last Updated: February 2026*
