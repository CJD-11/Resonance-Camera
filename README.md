# Resonance Camera

**Resonance Camera** is an experimental physical-computational imaging system that combines live camera imagery with measurements of environmental phenomena that are normally invisible.

The current system works across four complementary data layers:

* **USGS geomagnetic measurements** — ground-based magnetic-field vectors from the closest supported observatory
* **EPA RadNet** — geographically local or regional gamma-radiation measurements from the nearest configured monitor
* **NMDB** — regional neutron-monitor counts used as a proxy for secondary cosmic-ray activity at ground level
* **NOAA GOES** — proton and electron flux in the near-Earth space environment

The project does not attempt to show what radiation, magnetism, or particle activity literally “looks like.” Instead, it treats **measurement as material for visual translation**, combining scientific data with computational image-making.

---

## Current Data Model

```text
                     CAMERA LOCATION
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          ▼                ▼                ▼
   USGS Geomag        EPA RadNet          NMDB
 ground magnetic      local/regional    regional cosmic-
     context            gamma data       ray neutron data
          │                │                │
          └──────────┬─────┴──────┬─────────┘
                     │            │
                     │            ▼
                     │        NOAA GOES
                     │       space-particle
                     │           flux
                     │            │
                     └──────┬─────┘
                            ▼
                      Raspberry Pi 4
                            │
                 normalization + caching
                            │
                      image mapping
                            │
                            ▼
                       visual output
```

Environmental measurements are retrieved independently from the live rendering loop and stored in a shared cache. The camera visualization reads only the latest in-memory snapshot, so network latency does not directly interrupt the live preview.

---

## Why the Project Moved Away From Onboard Sensors

Early prototypes used inexpensive onboard magnetometers and radiation sensors.

These components were valuable for testing the concept of direct environmental sensing, but their measurements were not stable or trustworthy enough to serve as the project’s primary data sources.

Magnetic readings were highly sensitive to:

* sensor orientation
* nearby electronics
* power wiring
* enclosure materials
* calibration

Radiation measurements introduced different limitations, including:

* long integration times
* low or inconsistent event counts
* sensor sensitivity
* difficulty validating readings against a calibrated reference

This led to a deliberate architectural shift:

> **Use established monitoring infrastructure for environmental measurement while keeping the interpretation and image-making process inside the camera.**

The original sensors remain useful for experimentation and comparison, but they are not treated as authoritative environmental measurements.

---

## Location-Responsive Data Selection

The software determines the camera location using:

1. explicit latitude / longitude configuration, or
2. approximate IP-based geolocation as a fallback

Explicit coordinates are recommended for field use.

The software then selects the most geographically relevant available sources:

* the nearest supported **USGS geomagnetic observatory**
* the nearest configured **EPA RadNet monitor**
* the nearest supported **NMDB neutron monitor**
* current **NOAA GOES** particle data

GOES differs from the other sources because it represents the broader near-Earth space environment rather than a measurement taken near the camera.

For a camera operating in Boston, the data hierarchy is approximately:

```text
Boston / camera location
        │
        ├── EPA RadNet Boston
        │      local environmental gamma context
        │
        ├── nearest compatible USGS observatory
        │      regional ground magnetic context
        │
        ├── NMDB Newark / Swarthmore
        │      regional secondary cosmic-ray context
        │
        └── NOAA GOES
               near-Earth proton / electron environment
```

A portrait made in Boston can therefore be influenced by environmental measurements relevant to that location without implying that a remote monitoring station directly measured radiation or particles striking the photographed individual.

---

## Visualization Modes

The environmental visualizations range from comparatively representational diagrams to intentionally abstract interpretations.

### Data-Informed / Schematic

**Field Lines**
Magnetic-vector direction and recent field variation control a dipole-like field projection.

**Particle Arrival**
GOES proton and electron activity controls incoming particle trajectories.

**Magnetosphere**
Combines geomagnetic measurements and space-particle activity into a schematic field-shell visualization.

**Local Radiation**
Nearby RadNet gamma measurements generate concentric interference structures around tracked bodies or the center of the frame.

**Cosmic Cascade**
NMDB neutron counts and GOES particle activity produce branching atmospheric shower-like traces.

### Abstract

**Signal Veil**
Layered membranes, discontinuities, and interruptions react to environmental activity.

**Resonance Map**
Topographic rings, gaps, spatial absences, and shifting contours respond to the incoming datasets.

**Data Bloom**
Environmental measurements radiate outward from tracked bodies or the center of the frame.

Earlier experimental visualization modes from the camera project are also retained in the repository.

---

## Color Treatments

Every visualization supports three global color modes.

### B&W + Color — Default

The photographic image is rendered in black and white while the environmental visualization remains in color.

### Full Color

Both the live camera image and visualization remain in color.

### Full B&W

The photographic image and visualization are both rendered monochromatically.

This allows the same visualization logic to move between documentary, graphic, and more abstract visual treatments without duplicating individual modes.

---

## Performance Strategy

Low-latency preview is a central design requirement.

The system separates tasks by update rate:

```text
Camera preview           up to ~30 fps target
Object detection         lower independent rate
USGS / GOES refresh      ~60 seconds
NMDB refresh             ~120 seconds
EPA RadNet refresh       ~5 minutes
```

Performance optimizations include:

* no API calls inside the frame-rendering loop
* cached environmental snapshots
* background network polling
* cached image-warp coordinate grids
* lightweight PIL / OpenCV overlays
* deterministic particle positioning instead of reallocating random effects every frame
* lower-resolution live preview
* higher-resolution saved captures

The result is a system where slow environmental measurements can influence a responsive live image without forcing the camera to operate at the speed of the APIs.

---

## Diagnostic Endpoint

The current environmental-data cache can be inspected at:

```text
http://<camera-address>:5000/environment_data
```

The JSON response includes:

* camera location
* selected USGS station and magnetic-field vector
* selected EPA RadNet monitor and gamma measurement
* selected NMDB monitor and neutron count rate
* NOAA GOES proton and electron flux
* source distance where applicable
* measurement timestamps
* API status and partial-data errors

This allows the data pipeline to be evaluated independently from the visual output.

---

## Hardware

| Component                                      | Role                                                |   Approx. Cost |
| ---------------------------------------------- | --------------------------------------------------- | -------------: |
| Raspberry Pi 4                                 | Primary computational platform                      |            $55 |
| Arduino-compatible microcontroller             | Hardware control / experimentation                  |            $15 |
| ArduinoCam camera module                       | Image capture                                       |            $30 |
| Magnetometer                                   | Experimental direct geomagnetic sensing             |            $10 |
| Radiation / particle sensor                    | Experimental direct radiation sensing               |            $60 |
| Environmental data services                    | Networked environmental measurements                |             $0 |
| Regulated power supply                         | System power                                        |            $20 |
| Custom 3D-printed enclosure + fabricated parts | Housing, mounting, internal structure, and assembly |           $100 |
| Wiring, connectors & hardware                  | Internal assembly                                   |            $20 |
| **Estimated Prototype Total**                  |                                                     | **≈ $310 USD** |

The magnetometer and radiation sensor were used during early prototyping and remain useful for experimentation, but the current architecture relies primarily on established monitoring datasets.

---

## Camera Body + Fabrication

The camera body was developed as a multi-part CAD assembly measuring approximately **160 × 110 × 89 mm**.

The enclosure evolved through repeated cycles of:

```text
CAD → print → assemble → test → revise
```

Physical prototyping exposed constraints including:

* component clearance
* cable and wiring volume
* mating tolerances
* wall thickness
* print orientation
* fastener placement
* assembly order
* service access

The enclosure therefore developed alongside the electronics and software rather than as a decorative shell added after the technical system was complete.

---

## Interpretation + Limitations

Resonance Camera is an **artistic and experimental imaging instrument**, not a calibrated scientific, medical, radiation-safety, dosimetry, or navigation instrument.

EPA RadNet, NMDB, NOAA GOES, and USGS measurements are produced by different instruments operating at different geographic and temporal scales.

The project intentionally preserves these distinctions.

The final visual output should therefore be understood as a **data-informed interpretation of an environment**, not as a literal image of magnetic fields, radiation, cosmic rays, or particles surrounding a photographed subject.

---

## Data Sources

**U.S. EPA RadNet**
https://www.epa.gov/radnet/radnet-near-real-time-air-data

**USGS Geomagnetism Web Service**
https://www.usgs.gov/tools/web-service-geomagnetism-data

**NMDB Neutron Monitor Database**
https://www.nmdb.eu/

**NOAA Space Weather Prediction Center — GOES Data**
https://services.swpc.noaa.gov/json/goes/

NMDB data should be acknowledged according to NMDB and individual station-provider requirements. See `DATA_SOURCES.md` for implementation and attribution details.

---

## Author

**CJD-11**

https://github.com/CJD-11

