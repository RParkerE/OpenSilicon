# R-FPM , Fourier Ptychography Microscope

A sub-500 nm resolution metrology instrument built for lithography feature
verification using Fourier Ptychographic Microscopy (FPM). Every acquisition
runs full FPM reconstruction to achieve synthetic-aperture resolution beyond
the conventional Abbe diffraction limit. The system supports two optical
geometries selected at acquisition time:

| Mode | LED position | Sample type | Phase output |
|------|-------------|-------------|--------------|
| **Reflection** | Above sample via beam splitter | Opaque / reflective (silicon, metal film) | Surface height: `h = φλ/(4π)` |
| **Transmission** | Below sample, shining up | Transparent (glass, quartz, photoresist) | Optical thickness: `h = φλ/(2π(n−1))` |

The reconstruction algorithm (Gerchberg-Saxton iterative phase retrieval)
is identical in both modes. Only the physical LED position and the
phase-to-height interpretation differ.

---

## Table of Contents

1. [Theory and Mathematics](#1-theory-and-mathematics)
2. [Operating Modes](#2-operating-modes)
3. [System Architecture](#3-system-architecture)
4. [Bill of Materials](#4-bill-of-materials)
5. [Hardware Assembly](#5-hardware-assembly)
6. [Software Setup](#6-software-setup)
7. [Configuration Reference](#7-configuration-reference)
8. [Code Module Reference](#8-code-module-reference)
9. [Acquisition Workflow](#9-acquisition-workflow)
10. [Image Quality Assessment](#10-image-quality-assessment)
11. [Reconstruction Algorithm](#11-reconstruction-algorithm)
12. [Calibration Procedure](#12-calibration-procedure)
13. [Target Resolution Budget](#13-target-resolution-budget)

---

## 1. Theory and Mathematics

### 1.1 Fourier Ptychography , Conceptual Foundation

Standard optical microscopy is bounded by the **Abbe diffraction limit**:

```
d = λ / (2 · NA)
```

where `d` is the minimum resolvable feature, `λ` is illumination wavelength,
and `NA` is the objective numerical aperture. For this system at λ = 395 nm
and NA = 0.25:

```
d_conventional = 395 nm / (2 × 0.25) = 790 nm
```

This is insufficient for sub-500 nm lithography verification. FPM overcomes
this limit by synthesizing a **larger effective aperture** from a sequence of
low-resolution images captured under angularly diverse illumination.

Each LED in the 5×5 array illuminates the sample from a different angle,
shifting the object's spatial frequency content into the passband of the
objective. The iterative reconstruction stitches these shifted spectral
patches together in Fourier space, recovering spatial frequencies the
objective cannot capture under on-axis illumination alone.

### 1.2 Illumination Wave Vector

For LED at grid position `(m, n)` where `m, n ∈ {-2, -1, 0, 1, 2}`, the
physical position on the array is:

```
x_led = m · d_pitch      (meters)
y_led = n · d_pitch
```

The illumination angle at the sample plane:

```
θ_x = arctan(x_led / h)
θ_y = arctan(y_led / h)
```

where `h` is the vertical distance from LED array to sample (led_height_mm).
The incident wave vector components:

```
k_x = k₀ · sin(θ_x)
k_y = k₀ · sin(θ_y)
k₀  = 2π / λ            (wave number)
```

The small-angle approximation `sin(θ) ≈ tan(θ)` is valid for `θ < 15°`.
For this system with `d_pitch = 8 mm` and `h = 50 mm`, the corner LED
angle is `arctan(2√2 × 8 / 50) ≈ 24°`, so the full sine is used.

### 1.3 Fourier Space Shift (Pixel Units)

Reconstruction is performed in discrete Fourier space. Each wave
vector is converted to a pixel shift within the upsampled spectrum:

```
Δk_x_pixels = k_x / δk_x
Δk_y_pixels = k_y / δk_y
```

where the Fourier sampling interval is:

```
δk_x = 2π / (W · p_eff)
δk_y = 2π / (H · p_eff)
```

`W`, `H` are the image width and height in pixels, and `p_eff` is the
effective pixel size at the sample plane:

```
p_eff = p_sensor / M
p_sensor = 1.12 μm    (IMX219 native pixel pitch)
M        = 10         (objective magnification)
p_eff    = 0.112 μm = 112 nm
```

### 1.4 Synthetic Numerical Aperture

The synthetic NA after FPM reconstruction is:

```
NA_synthetic = NA_obj + NA_illumination
NA_illumination = sin(θ_max)
θ_max = arctan(√2 · 2 · d_pitch / h)    (corner LED)
```

For this system:

```
θ_max = arctan(√2 × 2 × 0.008 / 0.050) = arctan(0.452) ≈ 24.3°
NA_illumination ≈ sin(24.3°) ≈ 0.412
NA_synthetic    ≈ 0.25 + 0.412 = 0.662
```

Theoretical synthetic resolution:

```
d_synthetic = λ / (2 · NA_synthetic)
            = 395 nm / (2 × 0.662)
            ≈ 298 nm
```

This is well within the sub-500 nm target.

> **Requirement for this to hold:** Adjacent LED spectra must overlap in
> Fourier space by at least 30%. Overlap fraction is:
>
> ```
> overlap = 1 - (|Δk| / (2 · k_obj_radius))
> k_obj_radius = k₀ · NA_obj
> ```
>
> If `|Δk| > 2 · k_obj_radius`, spectra do not overlap and the
> Gerchberg-Saxton update is operating blind (no spectral continuity
> constraint). The primary failure mode to check during calibration.

### 1.5 Gerchberg-Saxton Phase Retrieval

The reconstruction is based on the
Gerchberg-Saxton (GS) algorithm adapted for FPM by Zheng et al. (2013).

**State:** A complex-valued high-resolution object spectrum
`O(k_x, k_y)` of shape `(N·U, M·U)` where `U` is the upsample factor
(default 4×).

**Per-iteration update for each LED `i`:**

1. Extract the spectral patch centered at `(Δk_xi, Δk_yi)`:
   ```
   ψ_i(k) = O(k + Δk_i) · P(k)
   ```
   where `P(k)` is the binary pupil function (objective aperture).

2. Inverse FFT to spatial domain:
   ```
   φ_i(r) = ℱ⁻¹{ψ_i(k)}
   ```

3. **Amplitude constraint** , replace amplitude with square root of
   measured intensity, preserve phase:
   ```
   φ_i'(r) = √I_i(r) · exp(j · arg(φ_i(r)))
   ```

4. Forward FFT back to Fourier domain:
   ```
   ψ_i'(k) = ℱ{φ_i'(r)}
   ```

5. Update high-resolution spectrum:
   ```
   O'(k + Δk_i) = O(k + Δk_i) · (1 - P(k)) + ψ_i'(k) · P(k)
   ```

**Convergence metric** (relative change between iterations):

```
ε = ‖O_new - O_old‖_F / ‖O_old‖_F
```

Iteration stops when `ε < convergence_threshold` (default 1×10⁻⁴) or
`max_iterations` is reached (default 15).

**Final reconstruction:**

```
object(r) = ℱ⁻¹{O(k)}
amplitude  = |object(r)|
phase      = arg(object(r))
```

### 1.6 Pupil Function

The objective lens is modeled as a binary circular aperture:

```
P(k) = 1   if √(k_x² + k_y²) ≤ NA_obj / λ
P(k) = 0   otherwise
```

The coherent transfer function (CTF) of a diffraction-limited
objective in the paraxial approximation (Born & Wolf, §10.6).

### 1.7 Phase-to-Height Conversion (Mode-Dependent)

The phase output of the GS reconstruction, `φ(r) ∈ [−π, π]`, encodes a
physical quantity that depends on the optical geometry.

**Reflection mode** (light travels to the surface and back , double pass):

```
φ = (4π / λ) · h
→  h = φλ / (4π)
```

`h` is the surface height in nm. Sensitivity is `λ/2 = 197 nm` per `2π`
of phase, so one full phase wrap corresponds to ~197 nm of topography.

**Transmission mode** (light passes through the sample once , single pass):

```
φ = (2π / λ) · (n − 1) · t
→  t = φλ / (2π(n − 1))
```

`t` is the physical thickness of the sample in nm. `n` is the sample
refractive index (configured via `sample_refractive_index` in `config.py`).
Sensitivity is `λ / (n − 1)` per `2π` of phase , for glass (`n ≈ 1.5`),
one full phase wrap corresponds to ~790 nm of thickness.

**Refractive index reference values:**

| Material | n at 395 nm |
|----------|-------------|
| Borosilicate glass | 1.47 |
| Fused silica (quartz) | 1.47 |
| Positive photoresist (AZ) | 1.64 |
| PMMA | 1.50 |
| Water | 1.34 |
| Air (reference) | 1.00 |

The `visualize.py` dashboard applies the correct formula automatically by
reading the mode from `status.json`. The refractive index can be overridden
interactively in the 3D surface tab without re-running reconstruction.

### 1.8 Abbe Resolution Limit Reference Table

| Configuration | NA | Resolution |
|---|---|---|
| Conventional (this objective) | 0.25 | 790 nm |
| FPM synthetic (estimated) | 0.662 | 298 nm |
| Target | , | < 500 nm |
| IMX219 Nyquist at sample plane | , | 224 nm |

The sensor Nyquist limit at the sample plane (`2 × p_eff = 224 nm`) is the
hard floor below which no reconstruction can recover detail regardless of NA.
The synthetic resolution (298 nm) is above this floor, so the sensor is not
the bottleneck.

---

## 2. Operating Modes

Both modes run full FPM reconstruction. The mode controls only where the
LED array is physically mounted and how the phase output is interpreted.

### 2.1 Reflection Mode

```
         [ 5×5 UV LED array ]
                  │  angled illumination
                  ▼
         [ 50/50 Beam Splitter ]  ← mounted at 45°
                  │  reflected arm
                  ▼
            [ Sample plane ]       ← opaque / reflective
                  │  backscattered light
                  ▼
          [ 10× Objective ]
                  │
                  ▼
           [ IMX219 Camera ]
```

**Use for:** Silicon wafers, metal thin films, hard disk platters, any
surface that scatters or reflects light without being transmissive.

**Phase output:** Surface topography. Each pixel's phase value encodes how
much higher or lower that point is relative to the mean surface plane.

**Beam splitter requirement:** The 46×57×0.5 mm 50/50 plate is in the
optical path. It introduces a 50% transmission loss on both the
illumination and collection passes (25% total efficiency). The
primary reason exposure times are long in reflection mode.

### 2.2 Transmission Mode

```
            [ Sample plane ]       ← transparent (glass, resist, quartz)
                  │  transmitted light
                  ▼
          [ 10× Objective ]
                  │
                  ▼
           [ IMX219 Camera ]

         [ 5×5 UV LED array ]     ← mounted BELOW sample, shining up
```

**Use for:** Thin film samples on transparent substrates, patterned
photoresist on glass, biological specimens on glass slides, any sample
that transmits UV light.

**Phase output:** Optical path length through the sample. Combined with
the refractive index, this gives physical thickness at each pixel.

**Beam splitter requirement:** Not needed. Remove it from the optical path
or fold it out of the beam. Transmission efficiency is significantly
higher , exposures can be 5–10× shorter than reflection mode for the
same SNR.

**LED array repositioning:** The LED jig must be moved from above the
beam splitter to below the sample stage. The `led_height_mm` parameter
in `config.py` must be updated to the new measured distance from the
LED array plane to the sample underside.

### 2.3 What Does Not Change Between Modes

- The GS reconstruction algorithm (`reconstruction.py`) , identical.
- The k-space coordinate calculation (`utils.py`) , identical. Illumination
  angle geometry is the same whether the LED is above or below.
- The `val.py` quality assessment , identical. Alignment, exposure, and
  focus checks apply equally in both modes.
- The session file format , identical. `led_NN.png`, `status.json`, and
  the TIFF outputs have the same structure.

### 2.4 Selecting a Mode

```bash
# Reflection mode (default) , opaque sample, beam splitter in path
python client.py --mode reflection

# Transmission mode , transparent sample, LED below stage
python client.py --mode transmission

# Set default in .env to avoid passing --mode every time
echo "FPM_MODE=transmission" >> .env
```

The mode is stored in the session manifest at capture time and in
`status.json` at reconstruction time. The `visualize.py` dashboard reads
it automatically , no manual configuration is needed when browsing results.

---

## 3. System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 4B                           │
│                                                             │
│   client.py ─── picamera2 ─── IMX219 sensor                │
│       │                            │                        │
│       │ pyserial (115200 baud)     │ CSI-2 lane             │
│       │                            ▼                        │
│       │                       led_XX.png                    │
│       │                            │                        │
│       │                      HTTP POST /upload              │
│       │                            │                        │
└───────┼────────────────────────────┼────────────────────────┘
        │                            │
        ▼                            ▼
┌──────────────┐           ┌─────────────────────────────────┐
│   ESP32      │           │     Server (Docker / bare)      │
│              │           │                                 │
│ SPI → 4×    │           │   server.py (FastAPI)           │
│ 74HC595     │           │   reconstruction.py (GS)        │
│ shift regs  │           │   utils.py (k-space coords)     │
│             │           │   visualize.py (Streamlit)      │
│ 25 UV LEDs  │           │   val.py (QA tool, local)       │
│ 5×5 grid    │           │                                 │
└──────────────┘           │   :8000 REST API               │
                           │   :8501 Dashboard              │
                           └─────────────────────────────────┘
```

**Optical path , Reflection mode** (LED above sample, beam splitter required):

```
  UV LED array (395 nm)
        │  angled illumination
        ▼
  ┌─────────────┐
  │ Beam Splitter│  46×57×0.5mm, 50/50 ratio
  └─────┬───────┘
        │ reflected down onto sample
        ▼
    Sample plane  (opaque/reflective)
        │ backscattered light up through splitter
        ▼
  10× Objective (NA 0.25)
        │
        ▼
  IMX219 Camera
```

**Optical path , Transmission mode** (LED below sample, no beam splitter):

```
  IMX219 Camera
        │
        ▼
  10× Objective (NA 0.25)
        │
        ▼
    Sample plane  (transparent)
        │ transmitted light
        ▼
  UV LED array (395 nm)  ← mounted below stage
```

### 3.1 Data Flow Per Acquisition

```
python client.py --mode <reflection|transmission>
    → ESP32 serial "L{n} E{ms}"
    → LED n fires for exposure_ms
    → Pi camera captures frame
    → saved as led_NN.png
    → repeated for all 25 LEDs
    → mode written to session manifest JSON
    → 25 images uploaded to server
    → POST /process/{session_id}?mode=<reflection|transmission>
    → mode stored in status.json
    → reconstruction runs (identical algorithm in both modes)
    → amplitude + phase TIFFs written to session dir
    → Streamlit dashboard reads mode from status.json
    → applies correct phase-to-height formula for the mode
```

---

## 4. Bill of Materials

| # | Component | Part / Spec | Role |
|---|-----------|-------------|------|
| 1 | **Objective Lens** | OMAX 10× Achromatic, NA 0.25, RMS thread, 160mm tube length | Primary imaging element. Sets base resolution and field of view. |
| 2 | **Microcontroller** | ESP32 (38-pin DevKit, dual-core 240 MHz, Wi-Fi/BT) | LED array sequencing via SPI + 4× shift registers. Serial command interface to Pi. |
| 3 | **UV LED Array** | EDGELEC 3mm, 395–400 nm, flat lens, DC 3V, ~20 mA/LED. 25 pcs arranged in 5×5 grid on 8 mm pitch PCB/jig | Angularly diverse illumination source. UV chosen for shortest λ → best diffraction limit. |
| 4 | **Camera** | Arducam IMX219 8MP, RPi NoIR (no IR filter), CSI-2 | Scientific imaging sensor. 3280×2464 native, 1.12 μm pixel pitch. NoIR variant avoids IR-cut filter UV absorption. |
| 5 | **Beam Splitter** | Optical glass, 46×57×0.5mm, 50/50 split ratio | Directs LED illumination onto sample in reflection geometry while passing return light to camera. |
| 6 | **Shift Registers** | SN74HC595N, 8-bit, DIP-16, 3-state output. 4 pcs in daisy-chain (32 outputs, 25 used) | Expands ESP32 SPI to 25 individual LED control lines. |
| 7 | **SBC** | Raspberry Pi 4B (4GB RAM recommended) | Acquisition host: runs client.py, picamera2, serial comms. |

### 4.1 Supplementary Materials

| Item | Specification | Notes |
|------|---------------|-------|
| LED current-limiting resistors | 100 Ω for 6–12V supply; included in EDGELEC kit | Verify drop across LED (~3.2V forward) at 20 mA |
| Sample slide | Quartz preferred; plastic usable with <8s exposure | See §12 fluorescence note |
| Tube adapter | RMS-to-C-mount or RMS-to-M12 for IMX219 board | Mechanical coupling between objective and sensor |
| Optical rail / cage system | 30mm or 60mm cage, Thorlabs or equivalent | Rigidity critical , any vibration aliases phase error |
| LED jig/PCB | 5×5 matrix, 8mm pitch, 3mm LED holes | Custom or perf-board construction |
| DC power supply | 5V/3A for Pi; 9V/1A for LED array; 3.3V/5V for ESP32 | Separate supplies reduce ground noise |

---

## 5. Hardware Assembly

### 5.1 LED Array Wiring (ESP32 → 74HC595 Chain)

The four shift registers are daisy-chained: `Q7'` (serial out) of register N
connects to `DS` (serial in) of register N+1.

```
ESP32 GPIO 23 (MOSI) ──► DS   [SR1] Q7' ──► DS   [SR2] Q7' ──► DS   [SR3] Q7' ──► DS   [SR4]
ESP32 GPIO 18 (SCK)  ──► SHCP (all four, parallel)
ESP32 GPIO 5  (LATCH)──► STCP (all four, parallel)
GND ─────────────────────────────────────────────────────────────────────────────────── GND
```

`SPI.transfer32()` clocks 32 bits MSB-first. Bit 0 activates the output on
the last register in the chain (SR4 pin Q0). Bit 24 activates SR1 pin Q0.
Bits 25–31 are unused.

**LED index mapping** (1-based, matches `led_NN.png` filenames):

```
LED grid (physical, viewed from above):
Row 0 (top):   LED 1  LED 2  LED 3  LED 4  LED 5
Row 1:         LED 6  LED 7  LED 8  LED 9  LED 10
Row 2 (center):LED 11 LED 12 LED 13 LED 14 LED 15
Row 3:         LED 16 LED 17 LED 18 LED 19 LED 20
Row 4 (bottom):LED 21 LED 22 LED 23 LED 24 LED 25

Shift register bit assignment:
LED index n → bit (n-1) in the 32-bit SPI word
```

> **Critical:** Verify your physical wiring matches this mapping before
> the first acquisition. Fire LEDs 1, 5, 21, 25 (corners) and 13 (center)
> individually and confirm the centroid positions are as expected.
> See §11 Calibration Procedure.

### 5.2 Serial Command Protocol (ESP32)

Communication is ASCII over UART at 115200 baud, newline-terminated.

| Command | Response | Description |
|---------|----------|-------------|
| `L{n} E{ms}\n` | `PULSING_LED_INDEX:{n}` then `PULSE_COMPLETE` | Fire LED n (1-25) for ms milliseconds |
| `STATUS\n` | `LED_ACTIVE:{n}` or `LED_ACTIVE:0` | Query active LED |
| `RESET\n` | `SYSTEM_RESET` | Clear all registers, safe state |

Maximum pulse duration: 180,000 ms (3 minutes). The ESP32 auto-shutoff
timer fires `PULSE_COMPLETE` and extinguishes the LED when elapsed.

### 5.3 Optical Alignment

1. Mount beam splitter at 45° to both the illumination axis and the imaging axis.
2. Center the objective below the beam splitter exit port.
3. Place sample on the objective focal plane (~17.4 mm WD for this objective).
4. Align the LED array so LED 13 (center) illumination hotspot falls at the
   image center `(1024, 1024)` for a 2048×2048 capture.
5. The LED array plane should be parallel to the sample plane. Use a spirit
   level on the LED jig.
6. Target `led_height_mm = 50` ± 2 mm. Measure with calipers and update `config.py`.

---

## 6. Software Setup

### 6.1 Prerequisites

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv) package manager
- Docker + Docker Compose (for server deployment)
- Arduino IDE or `arduino-cli` (for ESP32 firmware)

### 6.2 Python Environment (Client + QA Tools)

```bash
# Clone the repository
git clone https://github.com/<your-org>/r-fpm.git
cd r-fpm

# Create virtual environment and install dependencies
uv venv
uv pip install -r requirements.txt

# Activate environment
source .venv/bin/activate        # Linux / macOS
.venv\Scripts\activate           # Windows
```

### 6.3 Server Deployment (Docker)

```bash
# Build and start the API server and Streamlit dashboard
docker compose up -d

# Verify health
curl http://localhost:8000/health

# View logs
docker compose logs -f api
```

The server exposes:
- `http://localhost:8000` , FastAPI REST API
- `http://localhost:8501` , Streamlit visualization dashboard

### 6.4 Environment Variables

Override any config value with the `FPM_` prefix:

```bash
# Example: switch to quartz target profile
export FPM_ACQUISITION__BF_EXPOSURE_US=60000000
export FPM_ACQUISITION__ANALOG_GAIN=8.0
export FPM_OPTICAL__LED_HEIGHT_MM=52.5   # measured value
```

### 6.5 ESP32 Firmware

Open `firmware/led_controller.ino` in Arduino IDE. Set board to
**ESP32 Dev Module**. Flash via USB. Verify with serial monitor at 115200 baud ,
`SHIFT_REGISTER_SYSTEM_READY` should print on boot.

---

## 7. Configuration Reference

All system parameters are defined in `config.py` using Pydantic settings.
Values can be overridden via environment variables or `.env` file.

### 7.1 SystemConfig (top-level)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `"reflection"` | Operating geometry. `"reflection"` = LED above sample via beam splitter; `"transmission"` = LED below sample. Override with `FPM_MODE=transmission`. |

### 7.2 OpticalConfig

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `wavelength_nm` | 395.0 | nm | LED center wavelength |
| `na_objective` | 0.25 | , | Objective numerical aperture |
| `pixel_size_um` | 1.12 | μm | IMX219 native pixel pitch |
| `magnification` | 10.0 | × | Objective magnification |
| `led_pitch_mm` | 8.0 | mm | LED-to-LED spacing in array |
| `led_height_mm` | 50.0 | mm | Distance from LED array to sample plane. **Remeasure and update when switching to transmission mode** , the LED array moves from above the beam splitter to below the sample stage. |
| `sample_refractive_index` | 1.5 | , | Sample refractive index for transmission mode thickness conversion. Has no effect in reflection mode. Typical values: glass/quartz 1.47, photoresist 1.64, water 1.34. |

**Derived properties** (read-only, computed at runtime):

| Property | Formula | Typical value |
|----------|---------|---------------|
| `pixel_size_m` | `pixel_size_um × 1e-6 / magnification` | 112 nm |
| `theoretical_resolution_nm` | `wavelength_nm / (2 × na_objective)` | 790 nm |
| `synthetic_na_estimate` | `na_obj + sin(arctan(√2 × 2 × pitch / height))` | 0.662 |
| `synthetic_resolution_nm` | `wavelength_nm / (2 × synthetic_na_estimate)` | ~298 nm |

### 7.3 AcquisitionConfig

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | ESP32 serial device |
| `camera_width` | 2048 | Capture width (pixels) |
| `camera_height` | 2048 | Capture height (pixels) |
| `bf_exposure_us` | 9,600,000 | Bright-field exposure (9.6 s) |
| `df_exposure_us` | 9,600,000 | Dark-field exposure (9.6 s) |
| `analog_gain` | 16.0 | Camera gain (1.0–16.0) |
| `capture_timeout_s` | 60 | Max wait per frame |

### 7.4 Exposure Profiles

| Profile | Target | bf_exposure_us | df_exposure_us | analog_gain | Signal |
|---------|--------|---------------|----------------|-------------|--------|
| Plastic target (current) | Below fluorescence threshold | 7,000,000 | 7,000,000 | 16.0 | 5–10% FS , marginal |
| **Quartz target (recommended)** | Maximum SNR | 60,000,000 | 120,000,000 | 8.0 | 30–60% FS , good |
| Blue LED 470 nm (best) | Fast + no fluorescence | 1,000,000 | 3,000,000 | 4.0 | 40–80% FS , excellent |

> **Fluorescence warning:** The plastic calibration slide exhibits
> UV-induced fluorescence (visible as a red/pink channel bleed in the
> camera raw) at exposures exceeding ~8–10 seconds. This contaminates
> the measured intensity with incoherent emission rather than coherent
> scattered signal, violating the FPM measurement model. Use a quartz
> slide for exposures above 8 s.

### 7.5 ReconstructionConfig

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `upsample_factor` | 4 | 2–8 | High-res grid size = input × upsample_factor |
| `max_iterations` | 15 | 1–100 | GS iteration limit |
| `convergence_threshold` | 1×10⁻⁴ | >0 | Early stopping relative change |
| `regularization_weight` | 0.01 | 0–1 | Tikhonov noise suppression |

---

## 8. Code Module Reference

### 8.1 `config.py` , System Configuration

Centralized Pydantic settings model. All physical constants live here.

**`OperatingMode`** is a `Literal["transmission", "reflection"]` type alias
exported at module level. All other modules import this type to ensure
consistent validation:

```python
from config import config, OperatingMode
```

**`SystemConfig.mode`** sets the system-wide default geometry. Override
at the environment level without touching code:

```bash
FPM_MODE=transmission python client.py
# or persistently:
echo "FPM_MODE=transmission" >> .env
```

**`OpticalConfig.sample_refractive_index`** (default 1.5) is consumed
only by `visualize.py` in transmission mode when converting phase to
physical thickness. It has no effect on the reconstruction algorithm.

```python
from config import config

print(config.mode)                              # 'reflection'
print(config.optical.synthetic_resolution_nm)  # ~298 nm
print(config.optical.sample_refractive_index)  # 1.5
config.print_summary()                         # prints mode-aware summary
```

### 8.2 `utils.py` , Mathematical Utilities

**`get_k_coordinates(led_pitch_m, led_height_m, wavelength_m, na_obj, pixel_size_m, img_shape)`**

Computes the Fourier-space pixel shifts for all 25 LED positions.
Returns `(kx_shifts, ky_shifts)` as flat arrays of length 25.
These are the `Δk` values used in the GS update step.

**`generate_pupil(img_shape, na_obj, wavelength_m, pixel_size_m)`**

Returns a binary complex64 array representing the objective aperture
in Fourier space. Pixel value 1.0 inside NA cutoff, 0.0 outside.

**`calculate_theoretical_resolution(wavelength_m, na_effective)`**

Returns Abbe limit `λ / (2 × NA)` in meters.

**`estimate_synthetic_na(na_obj, led_pitch_m, led_height_m, wavelength_m)`**

Returns estimated synthetic NA based on LED geometry. Includes overlap
check: if adjacent spectra do not overlap, logs a warning.

### 8.3 `reconstruction.py` , FPM Reconstruction Engine

```python
from config import config
from reconstruction import FPMReconstructor

reconstructor = FPMReconstructor(config)
amplitude, phase, metadata = reconstructor.reconstruct(images, kx_shifts, ky_shifts)
```

**`FPMReconstructor.reconstruct(images, kx_shifts, ky_shifts)`**

- `images`: List of 25 float32 grayscale arrays
- `kx_shifts`, `ky_shifts`: Arrays from `get_k_coordinates()`
- Returns: amplitude array, phase array (radians), metadata dict

The metadata dict contains convergence history, processing time, and
the final relative change value. Useful for diagnosing reconstruction
quality without inspecting the output image.

### 8.4 `server.py` , FastAPI REST Server

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | System health, default mode, and optical parameters |
| `/sessions` | GET | List all acquisition sessions |
| `/sessions/{id}/status` | GET | Reconstruction progress and mode |
| `/upload/{id}` | POST | Upload image stack (multipart) |
| `/process/{id}?mode=reflection` | POST | Trigger reconstruction with specified mode |

**`POST /process/{session_id}?mode=<mode>`** accepts `mode` as a query
parameter. The mode defaults to `config.mode` if omitted. It is stored in
`status.json` immediately (even before reconstruction completes) so the
visualizer can display the correct formula while processing is in progress.

**`GET /sessions/{id}/status`** response now includes a `"mode"` field:

```json
{
  "session_id": "fpm_1771347801",
  "status": "complete",
  "mode": "transmission",
  "frames_uploaded": 25,
  "reconstruction_complete": true
}
```

### 8.5 `client.py` , Raspberry Pi Acquisition Client

Runs on the Raspberry Pi. Sequences through all 25 LEDs, captures one
image per LED, and uploads the stack to the server.

**`--mode` flag** is the primary user-facing control for selecting the
optical geometry. It defaults to `config.mode` (set in `.env`):

```bash
python client.py --mode reflection   # LED above, beam splitter in path
python client.py --mode transmission # LED below, beam splitter removed
```

The mode is propagated through the full pipeline:
- Written into `{session_id}_manifest.json` at capture time (per-frame)
- Sent as `?mode=` query parameter to `POST /process/{session_id}`
- The server stores it in `status.json` for the visualizer to read

**`FPMConductor`** now takes `mode: OperatingMode` as a constructor
argument. The BF/DF LED split logic is identical in both modes , inner
3×3 LEDs always use `bf_exposure_us`, outer 16 use `df_exposure_us`.

### 8.6 `visualize.py` , Streamlit Dashboard

Runs at `:8501`. Reads the operating mode from `status.json` (written by
server) or the session manifest (written by client) and applies the
correct phase-to-height formula automatically.

**`load_session_mode(session_path)`** reads mode with a three-level fallback:
1. `status.json` → `"mode"` field (primary , written at reconstruction time)
2. `{session_id}_manifest.json` → `"mode"` field (written at capture time)
3. `config.mode` (fallback for sessions captured before mode tracking)

**`phase_to_height(phase_map, wavelength_nm, mode, refractive_index)`**
applies the correct conversion formula:

```python
# Reflection , double-pass surface height
height_nm = (phase_rad * wavelength_nm) / (4 * pi)

# Transmission , single-pass optical thickness
height_nm = (phase_rad * wavelength_nm) / (2 * pi * (n - 1))
```

In transmission mode the dashboard exposes a live `n` override slider in
the 3D surface tab, allowing immediate rescaling of the height axis for
different sample materials without re-running reconstruction.

**Mode badge** is displayed in both the sidebar and the main area so the
geometry is unambiguous when switching between sessions of different modes.

Provides:
- Session browser (sorted by modification time)
- 16-bit amplitude and phase image display
- Interactive 3D surface plot (Plotly) with quality slider
- RMS roughness, mean height, and peak-to-valley metrics

### 8.7 `val.py` , Image Quality Assessment Tool

Standalone diagnostic tool. Runs locally on the acquisition machine or
server. Evaluates each of the 25 LED images across three independent axes:

| Axis | Method | Metric | PASS threshold |
|------|--------|--------|---------------|
| **Alignment** | Intensity-weighted centroid of top-5% pixels | Distance from image center (px) | < 100 px |
| **Exposure** | Mean intensity + clipping fraction | % of full scale | 8–80% |
| **Focus** | Laplacian variance of central 50% ROI | Lap. var (unitless) | > 15.0 |

```bash
# Run quality assessment on a session
python val.py data/fpm_1771347801

# Save PNG report to custom path
python val.py fpm_1771347801 --output ./reports/session_qa.png
```

**Output files per session:**
- `alignment_analysis.png` , 3×2 quality dashboard
- `alignment_results.json` , Full per-LED metrics + thresholds

#### Focus Threshold Calibration

The `FOCUS_PASS = 15.0` threshold is a starting point. Calibrate it
for your specific sample and objective:

1. Capture a single image with the system in known-good focus.
2. Run `val.py` and note the `laplacian_variance` for that LED.
3. Set `FOCUS_PASS` to ~80% of that value in `val.py`.

For a blank slide (no sample), the Laplacian variance will be near zero
regardless of focus , this is expected. The focus metric only works when
the sample has spatial features at or near the sensor resolution limit.

---

## 9. Acquisition Workflow

Every acquisition runs full FPM reconstruction. The only choice is the
optical geometry , reflection or transmission , which determines where
the LED array sits and what the phase output means.

### 9.1 Reflection Mode Acquisition

LED array above sample via beam splitter. Use for opaque/reflective samples.

```bash
# 1. Ensure beam splitter is in the optical path.
#    LED array should be above the beam splitter at led_height_mm from sample.

# 2. Start server (if not already running)
docker compose up -d

# 3. On Raspberry Pi: run full 25-LED acquisition
python client.py --mode reflection --session fpm_$(date +%s)

# 4. Monitor reconstruction status
curl http://<server_ip>:8000/sessions/<session_id>/status

# 5. View results in dashboard (mode badge shown automatically)
open http://<server_ip>:8501
```

### 9.2 Transmission Mode Acquisition

LED array below sample, shining up through specimen. Use for transparent
samples. Before running:

1. **Remove or fold the beam splitter** out of the optical path between
   the objective and camera.
2. **Reposition the LED array** below the sample stage, shining upward.
3. **Measure the new `led_height_mm`** (LED array plane to sample underside)
   with calipers and update `config.py` or set `FPM_OPTICAL__LED_HEIGHT_MM`.

```bash
# Set new height if different from 50 mm
export FPM_OPTICAL__LED_HEIGHT_MM=42.5

# Run acquisition
python client.py --mode transmission --session fpm_$(date +%s)

# View results , dashboard applies transmission formula automatically
open http://<server_ip>:8501
```

### 9.3 Setting a Default Mode

To avoid passing `--mode` on every run, set it in `.env`:

```bash
echo "FPM_MODE=transmission" >> .env
# Now client.py, server.py, and visualize.py all default to transmission
```

### 9.4 LED Firing Order

LEDs fire in index order 1→25 (row-major, top-left to bottom-right).
The angular diversity and k-space shift mapping are identical in both
reflection and transmission modes.

BF LEDs (inner 3×3, indices 7, 8, 9, 12, 13, 14, 17, 18, 19): use
`bf_exposure_us`. DF LEDs (outer 16): use `df_exposure_us`.

---

## 10. Image Quality Assessment

Before triggering reconstruction, run `val.py` to verify the stack is
usable. The tool will flag any LED whose image fails on alignment, exposure,
or focus, and will print a per-LED table and an overall readiness verdict.

### 10.1 Interpreting the Table

```
LED   OVERALL   ALIGN   DIST    EXPOSE   MEAN%   CLIP%   FOCUS   LAP_VAR
1     PASS      PASS    45.2px  PASS     31.4%   0.00%   PASS     24.55
2     WARN      WARN    187.0px PASS     28.1%   0.00%   PASS     19.22
  -> ALIGN WARN: centroid 187px from center (PASS < 100px).
3     FAIL      PASS    62.0px  FAIL     3.1%    0.00%   PASS     21.33
  -> EXPOSURE FAIL: mean 3.1% , severely underexposed. Increase exposure.
```

### 10.2 Status Color Key

| Status | Meaning |
|--------|---------|
| **PASS** | Axis is within specification |
| **WARN** | Axis is marginal , reconstruction may succeed but quality is reduced |
| **FAIL** | Axis is out of specification , reconstruction likely to fail or produce artifacts |

### 9.3 Acquisition Readiness Verdict

| Verdict | Condition |
|---------|-----------|
| `READY` | Zero FAILs, ≤5 WARNs |
| `MARGINAL` | 1–3 FAILs , address before acquiring |
| `NOT READY` | >3 FAILs , do not reconstruct |

---

## 11. Reconstruction Algorithm

### 11.1 High-Resolution Grid Initialization

The high-resolution spectrum is initialized using the center LED (index 12,
0-based) image. This provides the initial estimate of the object spectrum in
the passband of the objective. All other spectral regions start at zero.

```
high_res_shape = (2048 × upsample_factor, 2048 × upsample_factor)
               = (8192, 8192) at 4× upsample
```

### 11.2 Iteration Sequence

```
for iteration in range(max_iterations):
    for each LED (25 total):
        extract patch → apply pupil → iFFT → replace amplitude → FFT → update spectrum
    compute ε (relative change)
    if ε < convergence_threshold: break
```

Each full pass through all 25 LEDs constitutes one iteration. Typical
convergence at 15 iterations with 9.6 s exposures (marginal SNR).
With quartz target at 60 s exposure, convergence is typical at 5–8 iterations.

### 11.3 Output Files

| File | Format | Content |
|------|--------|---------|
| `reconstruction_amplitude.tiff` | 16-bit grayscale TIFF | Normalized amplitude (0–65535) |
| `reconstruction_phase.tiff` | 16-bit grayscale TIFF | Phase mapped from \[-π, π\] → \[0, 65535\] |
| `status.json` | JSON | Session status, operating mode, frame count, convergence metrics |

**Phase-to-height conversion** in `visualize.py` , formula selected by mode:

```python
phase_radians = (phase_uint16 / 65535.0) * (2π) - π

# Reflection mode (double-pass surface topography)
height_nm = (phase_radians × wavelength_nm) / (4π)

# Transmission mode (single-pass optical thickness)
height_nm = (phase_radians × wavelength_nm) / (2π × (n - 1))
```

The mode is read from `status.json` automatically. No manual formula
selection is required when viewing results in the dashboard.

---

## 12. Calibration Procedure

### 12.1 LED Mapping Verification

Fire each LED individually via the ESP32 serial console and measure
the centroid shift. Compare against geometric prediction:

```bash
# On any machine with the session data
python val.py data/<session_id>
```

Expected centroid positions (approximate, at 50mm height, 8mm pitch):

| LED | Grid pos | Expected centroid (approx) |
|-----|----------|---------------------------|
| 13 (center) | (0, 0) | ~(1024, 1024) |
| 1 (TL corner) | (-2, -2) | ~(840, 840) |
| 5 (TR corner) | (+2, -2) | ~(1208, 840) |
| 21 (BL corner) | (-2, +2) | ~(840, 1208) |
| 25 (BR corner) | (+2, +2) | ~(1208, 1208) |

If centroids do not match, check:
1. `led_height_mm` , measure physically and update `config.py`
2. `led_pitch_mm` , measure center-to-center with calipers
3. Shift register wiring , confirm LED index to physical position mapping

### 12.2 Focus Calibration

1. Place a calibration target (USAF 1951 chart or lithography sample) on the stage.
2. Manually adjust focus by translating the objective or stage in Z.
3. Run `val.py` after each Z adjustment.
4. The `LAP_VAR` column increases monotonically as you approach focus.
5. Lock the focus at maximum `LAP_VAR` for the center LED (LED 13).
6. Record that value as your `FOCUS_PASS` baseline in `val.py`.

### 12.3 Exposure Calibration

Target 20–60% of full scale (51–153 DN out of 255) in the mean intensity
column of `val.py`. Adjust `bf_exposure_us` and `df_exposure_us` in
`config.py` until bright-field and dark-field LEDs both land in this range.

For the current plastic target with UV LEDs:
- Bright-field (center LEDs): target ~50–100 DN mean
- Dark-field (outer LEDs): target ~30–80 DN mean (less efficient coupling)
- **Hard limit at 7,000,000 μs (7 s) to avoid fluorescence**

For quartz target: no fluorescence limit applies. Target 80–150 DN mean
at 60–120 s exposure.

---

## 13. Target Resolution Budget

| Budget item | Value | Notes |
|-------------|-------|-------|
| Illumination wavelength | 395 nm | UV LED center |
| Objective NA | 0.25 | OMAX 10× achromat |
| Conventional resolution | 790 nm | Abbe limit at objective NA |
| LED illumination NA | 0.412 | Corner LED at 50mm height |
| Synthetic NA (estimated) | 0.662 | Sum: obj + illumination |
| Synthetic resolution (estimated) | **298 nm** | Abbe at synthetic NA |
| Sensor Nyquist at sample | 224 nm | 2 × 112 nm effective pixel |
| **Target** | **< 500 nm** | Lithography feature verification |
| **Margin** | +40% | 298 nm vs 500 nm target |

The 40% margin accounts for:
- Spectral overlap imperfection (typically reduces effective synthetic NA by 10–20%)
- Phase noise from low-SNR dark-field images
- Optical aberrations not modeled in the paraxial pupil function
- Residual alignment error between measured and predicted k-shifts

---

## References

- Zheng, G., Horstmeyer, R., & Yang, C. (2013). Wide-field, high-resolution Fourier
  ptychographic microscopy. *Nature Photonics*, 7(9), 739–745.
  https://doi.org/10.1038/nphoton.2013.187

- Born, M., & Wolf, E. (2019). *Principles of Optics* (7th ed.). Cambridge University Press.
  §10.6 (Coherent imaging), §8.6 (Pupil function).

- Ou, X., Horstmeyer, R., Yang, C., & Zheng, G. (2013). Quantitative phase imaging
  via Fourier ptychographic microscopy. *Optics Letters*, 38(22), 4845–4848.

- IMX219 Datasheet, Sony Semiconductor Solutions. CMOS image sensor, 8 MP, 1.12 μm pixel.

- SN74HC595 Datasheet, Texas Instruments. 8-bit shift register with output latches.
