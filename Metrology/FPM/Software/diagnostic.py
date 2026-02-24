"""
FPM Post-Hoc Diagnostic Tool

Reads an existing 25-image session from disk and produces specific,
quantitative adjustment instructions — no hardware required.

Distinct from the other tools:

    val.py          PASS / WARN / FAIL status per LED.
                    Tells you *what* is wrong.

    calibration.py  Live-hardware checks + auto-tune.
                    Requires ESP32 + camera connected.

    diagnose.py     (THIS FILE)
                    Reads any existing session and tells you *exactly*
                    what to change and by how much:
                        • "led_height_mm is actually 43.2 mm — config says 50.0"
                        • "shift LED array 4.2 mm right, 1.1 mm towards camera"
                        • "set bf_exposure_us = 28_800_000 (3.0× current)"
                        • "set df_exposure_us = 43_200_000 (4.5× current)"
                        • "focus OK — Laplacian score 24.3 (threshold 5.0)"

Three diagnostic axes
─────────────────────
1. GEOMETRY  — fit led_height_mm and LED-array lateral offset from all 25
               measured centroids using nonlinear least-squares.
               Inputs:  measured centroids (from bright-spot detection)
               Outputs: inferred height, inferred x/y array offset (mm),
                        residuals per LED, dominant error type.

2. EXPOSURE  — per-LED mean/clip statistics; separate BF and DF groups;
               recommend new bf_exposure_us and df_exposure_us.
               Flags fluorescence risk (plastic + UV + long exposure).

3. FOCUS     — Laplacian variance per LED on the central ROI.
               Identifies best-focused LED, worst, and spatial gradient
               (detects sample tilt or non-flat mounting).

Usage
─────
    python diagnose.py data/fpm_1771832995
    python diagnose.py data/fpm_1771832995 --no-plot
    python diagnose.py data/fpm_1771832995 --json-only

Outputs (written to session directory)
───────────────────────────────────────
    diagnosis_report.json   full structured data
    diagnosis_summary.txt   human-readable plain-text report
    diagnosis_plot.png      3×2 annotated visualisation (optional)
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# scipy for nonlinear least-squares geometry fit
try:
    from scipy.optimize import minimize
    _SCIPY = True
except ImportError:
    _SCIPY = False

from imgload import load_uv_image
from config import config as _global_config, SystemConfig

logger = logging.getLogger("FPMDiagnose")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# BF LED indices (1-based, row-major 5×5 grid, inner 3×3)
BF_LEDS = {7, 8, 9, 12, 13, 14, 17, 18, 19}

# Centre LED (1-based)
CENTRE_LED = 13

# Exposure target: centre of the acceptable window
_EXPOSURE_TARGET_PCT = 47.5          # % of 8-bit full scale
_EXPOSURE_FSR        = 255.0
_FLUORESCENCE_LIMIT_US = 7_000_000  # 7 s — plastic UV fluorescence onset

# Focus thresholds (Laplacian variance on central ROI)
_FOCUS_PASS = 5.0
_FOCUS_WARN = 2.0

# Geometry fit bounds
_HEIGHT_MIN_MM = 10.0
_HEIGHT_MAX_MM = 200.0


# ---------------------------------------------------------------------------
# Per-LED measurement
# ---------------------------------------------------------------------------

@dataclass
class LEDMeasurement:
    led_number: int           # 1–25
    is_bf: bool
    centroid_x: Optional[float]   # None if detection failed
    centroid_y: Optional[float]
    peak_intensity: Optional[float]
    mean_intensity: float
    laplacian_score: float
    centroid_ok: bool
    snr: float                # peak / (mean + eps)


def _bright_centroid(img: np.ndarray) -> Tuple[float, float, float]:
    """
    Intensity-weighted centroid of the top-5% brightest pixels.
    Returns (cx, cy, peak).  Raises ValueError if no signal.
    """
    threshold = np.percentile(img, 95)
    mask = img > threshold
    if not np.any(mask):
        raise ValueError("No bright region above 95th percentile")
    masked = img * mask.astype(np.float32)
    total  = masked.sum()
    if total < 1e-6:
        raise ValueError("Bright region has negligible intensity")
    rows = np.arange(img.shape[0]).reshape(-1, 1)
    cols = np.arange(img.shape[1]).reshape(1, -1)
    cy = float((masked * rows).sum() / total)
    cx = float((masked * cols).sum() / total)
    peak = float(img[mask].max())
    return cx, cy, peak


def _laplacian_score(img: np.ndarray) -> float:
    """Normalised Laplacian variance on the central 50% ROI."""
    h, w = img.shape
    roi = img[h // 4: 3 * h // 4, w // 4: 3 * w // 4].astype(np.float32)
    lap = cv2.Laplacian(roi, cv2.CV_32F)
    return float(lap.var())


def measure_session(
    session_path: Path,
) -> Tuple[List[LEDMeasurement], Tuple[int, int]]:
    """
    Load all 25 LED images and compute per-LED measurements.

    Returns:
        (measurements, image_shape) where image_shape is (height, width).
    """
    measurements: List[LEDMeasurement] = []
    image_shape: Optional[Tuple[int, int]] = None

    for led_number in range(1, 26):
        path = session_path / f"led_{led_number:02d}.png"
        if not path.exists():
            logger.warning(f"Missing: {path.name}")
            continue

        img = load_uv_image(path)
        if img is None:
            logger.warning(f"Could not read: {path.name}")
            continue
        img = img.astype(np.float32)

        if image_shape is None:
            image_shape = img.shape

        mean_val = float(img.mean())
        lap      = _laplacian_score(img)

        try:
            cx, cy, peak = _bright_centroid(img)
            centroid_ok  = True
        except ValueError:
            cx = cy = peak = None
            centroid_ok    = False

        snr = float(peak / (mean_val + 1e-6)) if peak is not None else 0.0

        measurements.append(LEDMeasurement(
            led_number     = led_number,
            is_bf          = led_number in BF_LEDS,
            centroid_x     = cx,
            centroid_y     = cy,
            peak_intensity = peak,
            mean_intensity = mean_val,
            laplacian_score= lap,
            centroid_ok    = centroid_ok,
            snr            = snr,
        ))

    if image_shape is None:
        raise RuntimeError("No readable images found in session")

    return measurements, image_shape


# ---------------------------------------------------------------------------
# AXIS 1 — Geometry diagnostic
# ---------------------------------------------------------------------------

@dataclass
class GeometryDiagnosis:
    status: str                           # GOOD / MARGINAL / POOR
    inferred_height_mm: Optional[float]   # None if fit failed
    config_height_mm: float
    height_error_mm: Optional[float]      # inferred − config
    array_offset_x_mm: float             # lateral offset of array from axis
    array_offset_y_mm: float
    centre_led_offset_x_px: float        # pixel offset of LED 13 from image centre
    centre_led_offset_y_px: float
    rms_residual_px: float               # after best-fit geometry
    max_residual_px: float
    worst_led: int
    fit_converged: bool
    dominant_error: str                  # "height" | "lateral" | "wiring" | "ok"
    adjustment_instructions: List[str]
    per_led_residuals: Dict[int, float]


def _led_grid_positions(pitch_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return (x_m, y_m) for LEDs 1–25 in row-major order."""
    idx = np.arange(-2, 3)
    xi, yi = np.meshgrid(idx, idx)
    return xi.flatten() * pitch_m, yi.flatten() * pitch_m


def _predicted_shifts_pixels(
    height_m: float,
    array_offset_x_m: float,
    array_offset_y_m: float,
    pitch_m: float,
    wavelength_m: float,
    pixel_size_m: float,
    img_shape: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute predicted centroid positions (absolute pixels) for all 25 LEDs.

    LED positions are shifted by (array_offset_x_m, array_offset_y_m) to
    model a laterally mis-centred array.
    """
    k0 = 2 * np.pi / wavelength_m
    x_m, y_m = _led_grid_positions(pitch_m)
    x_m = x_m + array_offset_x_m
    y_m = y_m + array_offset_y_m

    r   = np.sqrt(x_m**2 + y_m**2 + height_m**2)
    kx  = k0 * x_m / r
    ky  = k0 * y_m / r

    h_px, w_px = img_shape
    delta_kx   = 2 * np.pi / (w_px * pixel_size_m)
    delta_ky   = 2 * np.pi / (h_px * pixel_size_m)

    cx = w_px / 2.0 + kx / delta_kx   # absolute pixel position
    cy = h_px / 2.0 + ky / delta_ky

    return cx, cy


def _fit_geometry(
    measurements: List[LEDMeasurement],
    image_shape: Tuple[int, int],
    cfg: SystemConfig,
) -> Tuple[float, float, float, float, bool]:
    """
    Fit (height_mm, offset_x_mm, offset_y_mm) by minimising the sum of
    squared centroid residuals over all LEDs with valid detections.

    Returns:
        (height_mm, offset_x_mm, offset_y_mm, rms_residual_px, converged)

    Convergence is False when:
        - scipy is not available
        - fewer than 5 LEDs have valid centroids
        - the optimiser hit the height boundary (200 mm) — indicates the
          cost landscape is too flat or the wiring errors dominate.
          In that case height_mm is returned as None (caller handles it).
    """
    valid = [m for m in measurements if m.centroid_ok]
    if len(valid) < 5:
        return cfg.optical.led_height_mm, 0.0, 0.0, 999.0, False

    led_indices = [m.led_number - 1 for m in valid]
    meas_cx     = np.array([m.centroid_x for m in valid])
    meas_cy     = np.array([m.centroid_y for m in valid])

    pitch_m      = cfg.optical.led_pitch_m
    wavelength_m = cfg.optical.wavelength_m
    pixel_size_m = cfg.optical.pixel_size_m

    def residuals(params):
        h_m, ox_m, oy_m = params
        if h_m <= 0:
            return 1e9
        cx_pred, cy_pred = _predicted_shifts_pixels(
            h_m, ox_m, oy_m, pitch_m, wavelength_m, pixel_size_m, image_shape
        )
        cx_sel = cx_pred[led_indices]
        cy_sel = cy_pred[led_indices]
        return float(np.sum((meas_cx - cx_sel)**2 + (meas_cy - cy_sel)**2))

    if not _SCIPY:
        # Fallback: centre LED only for lateral offset, height unchanged
        centre = next((m for m in valid if m.led_number == CENTRE_LED), None)
        if centre:
            ox_m = (centre.centroid_x - image_shape[1] / 2.0) * pixel_size_m
            oy_m = (centre.centroid_y - image_shape[0] / 2.0) * pixel_size_m
        else:
            ox_m = oy_m = 0.0
        return cfg.optical.led_height_mm, ox_m * 1000, oy_m * 1000, 999.0, False

    h0  = cfg.optical.led_height_mm / 1000.0
    ox0 = 0.0
    oy0 = 0.0

    result = minimize(
        residuals,
        x0      = [h0, ox0, oy0],
        method  = "Nelder-Mead",
        options = dict(xatol=1e-6, fatol=1e-2, maxiter=5000),
        bounds  = [
            (_HEIGHT_MIN_MM / 1000.0, _HEIGHT_MAX_MM / 1000.0),
            (-0.05, 0.05),
            (-0.05, 0.05),
        ]
    )

    h_fit, ox_fit, oy_fit = result.x
    h_mm  = float(h_fit)  * 1000.0
    ox_mm = float(ox_fit) * 1000.0
    oy_mm = float(oy_fit) * 1000.0

    # If the fit hit the upper height boundary it means the optimiser
    # couldn't find a consistent geometry.  The most common cause is
    # wiring errors in more than half the LEDs, which makes the cost
    # landscape roughly flat w.r.t. height.  Return converged=False
    # so the caller can issue a more useful diagnosis.
    boundary_hit = h_mm >= (_HEIGHT_MAX_MM * 0.98)
    if boundary_hit:
        return h_mm, ox_mm, oy_mm, 999.0, False

    # Compute RMS residual at the fitted geometry
    cx_pred, cy_pred = _predicted_shifts_pixels(
        h_fit, ox_fit, oy_fit,
        pitch_m, wavelength_m, pixel_size_m, image_shape
    )
    cx_sel  = cx_pred[led_indices]
    cy_sel  = cy_pred[led_indices]
    resid   = np.sqrt((meas_cx - cx_sel)**2 + (meas_cy - cy_sel)**2)
    rms_px  = float(np.sqrt(np.mean(resid**2)))

    return h_mm, ox_mm, oy_mm, rms_px, result.success


def diagnose_geometry(
    measurements: List[LEDMeasurement],
    image_shape: Tuple[int, int],
    cfg: SystemConfig,
) -> GeometryDiagnosis:
    """
    Back-calculate LED array geometry from the full 25-image centroid set.

    The nonlinear least-squares fit estimates:
        h         — actual distance from LED array to sample (mm)
        offset_x  — lateral shift of the array from the optical axis (mm)
        offset_y  — same in the orthogonal axis

    A residual > ~20px after fitting indicates wiring errors (one or more
    LEDs firing at a position that does not match their firmware index).
    """
    h_inferred, ox_mm, oy_mm, rms_px, converged = _fit_geometry(
        measurements, image_shape, cfg
    )

    # Per-LED residuals — only meaningful when fit converged
    valid = [m for m in measurements if m.centroid_ok]
    per_led_residuals: Dict[int, float] = {}

    if valid and converged:
        cx_pred, cy_pred = _predicted_shifts_pixels(
            h_inferred / 1000.0,
            ox_mm / 1000.0,
            oy_mm / 1000.0,
            cfg.optical.led_pitch_m,
            cfg.optical.wavelength_m,
            cfg.optical.pixel_size_m,
            image_shape,
        )
        for m in valid:
            idx  = m.led_number - 1
            resid = float(np.sqrt(
                (m.centroid_x - cx_pred[idx])**2
                + (m.centroid_y - cy_pred[idx])**2
            ))
            per_led_residuals[m.led_number] = round(resid, 1)
    else:
        # Fit did not converge — compute residuals against config geometry
        # (at config height, zero offset) so we still get useful per-LED data
        cx_cfg, cy_cfg = _predicted_shifts_pixels(
            cfg.optical.led_height_mm / 1000.0,
            0.0, 0.0,
            cfg.optical.led_pitch_m,
            cfg.optical.wavelength_m,
            cfg.optical.pixel_size_m,
            image_shape,
        )
        for m in valid:
            idx  = m.led_number - 1
            resid = float(np.sqrt(
                (m.centroid_x - cx_cfg[idx])**2
                + (m.centroid_y - cy_cfg[idx])**2
            ))
            per_led_residuals[m.led_number] = round(resid, 1)

    max_resid = max(per_led_residuals.values()) if per_led_residuals else 0.0
    worst_led = max(per_led_residuals, key=per_led_residuals.get) if per_led_residuals else 0

    # Centre LED offset from image centre (always available, fit-independent)
    centre_meas = next((m for m in measurements if m.led_number == CENTRE_LED), None)
    ic_x = image_shape[1] / 2.0
    ic_y = image_shape[0] / 2.0
    if centre_meas and centre_meas.centroid_ok:
        c13_off_x = float(centre_meas.centroid_x - ic_x)
        c13_off_y = float(centre_meas.centroid_y - ic_y)
    else:
        c13_off_x = c13_off_y = 0.0

    h_error_mm = round(h_inferred - cfg.optical.led_height_mm, 1) if converged else None

    # Convert pixel offset to mm at sample plane
    c13_off_x_mm = c13_off_x * cfg.optical.pixel_size_m * 1000.0
    c13_off_y_mm = c13_off_y * cfg.optical.pixel_size_m * 1000.0

    # --- Classify dominant error and compose instructions ---
    instructions: List[str] = []
    dominant = "ok"

    if not converged:
        # The fit could not find a self-consistent geometry.
        # This nearly always means wiring errors in the majority of LEDs,
        # which makes all centroids wrong and the cost landscape flat.
        dominant = "wiring"

        # LED 13 lateral offset is still reliable (it fires, we measured it)
        if abs(c13_off_x_mm) > 0.5 or abs(c13_off_y_mm) > 0.5:
            dir_x = "right" if c13_off_x_mm > 0 else "left"
            dir_y = "down"  if c13_off_y_mm > 0 else "up"
            instructions.append(
                f"PHYSICAL: LED 13 centroid is ({c13_off_x:+.0f}, {c13_off_y:+.0f}) px"
                f" from image centre.  Move LED array"
                f" {abs(c13_off_x_mm):.1f} mm {dir_x}"
                f" and {abs(c13_off_y_mm):.1f} mm {dir_y}"
            )

        instructions.append(
            "GEOMETRY FIT FAILED — the height optimiser diverged to the"
            " boundary (200 mm).  This means the LED centroid pattern is"
            " inconsistent with any single physical geometry."
        )
        instructions.append(
            "  The most common cause is shift-register wiring errors that"
            " cause multiple firmware LED indices to fire at the same"
            " physical position, producing a scrambled centroid map."
        )
        instructions.append(
            "  STEP 1: Verify shift register wiring before re-running."
        )
        instructions.append(
            "  STEP 2: Fire LEDs individually to confirm index → position:"
        )

        # Identify the worst outliers to check first (highest raw residuals)
        bad_leds = sorted(
            per_led_residuals.items(), key=lambda kv: kv[1], reverse=True
        )[:8]
        for n, r in bad_leds:
            instructions.append(
                f"          picocom /dev/ttyUSB0 -b 115200  "
                f"→  L{n} E2000  (residual {r:.0f} px)"
            )
        instructions.append(
            "  STEP 3: Once wiring is corrected, re-acquire and re-run"
            " diagnose.py — height fit will converge."
        )

    else:
        # Fit converged — report lateral offset and height error
        if abs(c13_off_x_mm) > 0.5 or abs(c13_off_y_mm) > 0.5:
            dominant = "lateral"
            dir_x = "right" if c13_off_x_mm > 0 else "left"
            dir_y = "down"  if c13_off_y_mm > 0 else "up"
            instructions.append(
                f"PHYSICAL: Move LED array {abs(c13_off_x_mm):.1f} mm {dir_x}"
                f" and {abs(c13_off_y_mm):.1f} mm {dir_y}"
                f" to centre LED 13 on the optical axis"
            )

        if h_error_mm is not None and abs(h_error_mm) > 2.0:
            dominant = "height"
            direction = "closer to" if h_error_mm < 0 else "further from"
            instructions.append(
                f"CONFIG:   led_height_mm is actually {h_inferred:.1f} mm"
                f" (config says {cfg.optical.led_height_mm:.1f} mm,"
                f" error = {h_error_mm:+.1f} mm)."
                f"  Move array {direction} sample OR update config:"
            )
            instructions.append(
                f"          FPM_OPTICAL__LED_HEIGHT_MM={h_inferred:.1f}"
            )

        wiring_suspects = [n for n, r in per_led_residuals.items() if r > 50.0]
        if wiring_suspects:
            dominant = "wiring"
            instructions.append(
                f"WIRING:   {len(wiring_suspects)} LEDs have residuals > 50 px"
                f" after geometry correction: {sorted(wiring_suspects)}."
                f"  Verify individual firing:"
            )
            for n in sorted(wiring_suspects)[:6]:
                instructions.append(
                    f"          picocom /dev/ttyUSB0 -b 115200  →  L{n} E2000"
                )
            if len(wiring_suspects) > 6:
                instructions.append(
                    f"          (and {len(wiring_suspects) - 6} more — see full list above)"
                )

        if rms_px < 15.0 and not wiring_suspects and abs(c13_off_x_mm) < 0.5:
            instructions.append("Geometry OK — no physical adjustments needed")

    # Status classification
    if not converged:
        status = "POOR"
    elif rms_px < 15.0 and (h_error_mm is None or abs(h_error_mm) < 5.0):
        status = "GOOD"
    elif rms_px < 40.0 and (h_error_mm is None or abs(h_error_mm) < 15.0):
        status = "MARGINAL"
    else:
        status = "POOR"

    return GeometryDiagnosis(
        status               = status,
        inferred_height_mm   = round(h_inferred, 1) if converged else None,
        config_height_mm     = cfg.optical.led_height_mm,
        height_error_mm      = h_error_mm,
        array_offset_x_mm    = round(c13_off_x_mm, 2),
        array_offset_y_mm    = round(c13_off_y_mm, 2),
        centre_led_offset_x_px = round(c13_off_x, 1),
        centre_led_offset_y_px = round(c13_off_y, 1),
        rms_residual_px      = round(rms_px, 1),
        max_residual_px      = round(max_resid, 1),
        worst_led            = worst_led,
        fit_converged        = converged,
        dominant_error       = dominant,
        adjustment_instructions = instructions,
        per_led_residuals    = {k: round(v, 1) for k, v in per_led_residuals.items()},
    )


# ---------------------------------------------------------------------------
# AXIS 2 — Exposure diagnostic
# ---------------------------------------------------------------------------

@dataclass
class ExposureDiagnosis:
    status: str                         # GOOD / MARGINAL / POOR
    bf_mean_percent: float
    df_mean_percent: float
    bf_current_us: int
    df_current_us: int
    bf_recommended_us: Optional[int]
    df_recommended_us: Optional[int]
    bf_scale_factor: float
    df_scale_factor: float
    saturated_leds: List[int]
    dark_leds: List[int]
    fluorescence_risk: bool
    adjustment_instructions: List[str]
    per_led_mean_percent: Dict[int, float]


def diagnose_exposure(
    measurements: List[LEDMeasurement],
    cfg: SystemConfig,
) -> ExposureDiagnosis:
    """
    Compute per-group (BF / DF) exposure scale factors and recommend new values.

    Scale factor = target_mean / median(group_means).
    Applied to the current configured exposure time.
    """
    bf_means: List[float] = []
    df_means: List[float] = []
    saturated: List[int]  = []
    dark: List[int]       = []

    per_led_pct: Dict[int, float] = {}

    for m in measurements:
        pct = m.mean_intensity / _EXPOSURE_FSR * 100.0
        per_led_pct[m.led_number] = round(pct, 2)

        # Clip detection: any image where > 0.5% of pixels are at 255
        img_path = None   # not re-loading; use peak as proxy
        if m.peak_intensity is not None and m.peak_intensity >= 254:
            saturated.append(m.led_number)

        if pct < 5.0:
            dark.append(m.led_number)

        if m.is_bf:
            bf_means.append(pct)
        else:
            df_means.append(pct)

    bf_median = float(np.median(bf_means)) if bf_means else 0.0
    df_median = float(np.median(df_means)) if df_means else 0.0

    def _scale(current_us: int, measured_pct: float) -> Tuple[float, Optional[int]]:
        if measured_pct < 0.5:
            return 1.0, None
        scale = _EXPOSURE_TARGET_PCT / measured_pct
        # Cap at 8× to avoid absurdly long exposures
        scale = float(np.clip(scale, 0.1, 8.0))
        rec   = int(current_us * scale)
        # Only recommend if change is > 10%
        if abs(rec - current_us) / current_us < 0.10:
            return scale, None
        return scale, rec

    bf_scale, bf_rec = _scale(cfg.acquisition.bf_exposure_us, bf_median)
    df_scale, df_rec = _scale(cfg.acquisition.df_exposure_us, df_median)

    # Fluorescence risk: plastic UV target + recommended time > 7 s
    fluorescence_risk = (
        (bf_rec is not None and bf_rec > _FLUORESCENCE_LIMIT_US)
        or (df_rec is not None and df_rec > _FLUORESCENCE_LIMIT_US)
        or cfg.acquisition.bf_exposure_us > _FLUORESCENCE_LIMIT_US
    )

    # Cap recs at fluorescence limit if risk applies
    if fluorescence_risk and bf_rec and bf_rec > _FLUORESCENCE_LIMIT_US:
        bf_rec = _FLUORESCENCE_LIMIT_US
    if fluorescence_risk and df_rec and df_rec > _FLUORESCENCE_LIMIT_US:
        df_rec = _FLUORESCENCE_LIMIT_US

    instructions: List[str] = []

    if bf_rec:
        instructions.append(
            f"CONFIG:  bf_exposure_us = {bf_rec:,}"
            f"  ({bf_scale:.1f}× current {cfg.acquisition.bf_exposure_us:,} us"
            f",  BF median was {bf_median:.1f}% FSR)"
        )
        instructions.append(
            f"         FPM_ACQUISITION__BF_EXPOSURE_US={bf_rec}"
        )
    if df_rec:
        instructions.append(
            f"CONFIG:  df_exposure_us = {df_rec:,}"
            f"  ({df_scale:.1f}× current {cfg.acquisition.df_exposure_us:,} us"
            f",  DF median was {df_median:.1f}% FSR)"
        )
        instructions.append(
            f"         FPM_ACQUISITION__DF_EXPOSURE_US={df_rec}"
        )
    if fluorescence_risk:
        instructions.append(
            "WARNING: UV fluorescence risk (plastic target + UV LEDs)."
            "  Max safe exposure is 7 s.  To increase signal:"
        )
        instructions.append(
            "         • Replace plastic slide with quartz (allows 60–120 s)"
        )
        instructions.append(
            "         • Raise analog_gain to 16.0 first"
        )
        instructions.append(
            "         • Switch to 470 nm blue LEDs (no fluorescence, 10× faster)"
        )
    if saturated:
        instructions.append(
            f"SATURATED LEDs: {sorted(saturated)}"
            f" — reduce gain or shorten these exposures"
        )
    if dark:
        instructions.append(
            f"VERY DARK LEDs (< 5% FSR): {sorted(dark)}"
            f" — outer-ring DF LEDs with low coupling are expected;"
            f" consider 3–5× longer DF exposure"
        )
    if not bf_rec and not df_rec and not saturated:
        instructions.append("Exposure OK — no changes needed")

    # Status
    if bf_median >= 20 and bf_median <= 70 and df_median >= 10:
        status = "GOOD"
    elif bf_median >= 8 or df_median >= 5:
        status = "MARGINAL"
    else:
        status = "POOR"

    return ExposureDiagnosis(
        status                  = status,
        bf_mean_percent         = round(bf_median, 1),
        df_mean_percent         = round(df_median, 1),
        bf_current_us           = cfg.acquisition.bf_exposure_us,
        df_current_us           = cfg.acquisition.df_exposure_us,
        bf_recommended_us       = bf_rec,
        df_recommended_us       = df_rec,
        bf_scale_factor         = round(bf_scale, 2),
        df_scale_factor         = round(df_scale, 2),
        saturated_leds          = sorted(saturated),
        dark_leds               = sorted(dark),
        fluorescence_risk       = fluorescence_risk,
        adjustment_instructions = instructions,
        per_led_mean_percent    = per_led_pct,
    )


# ---------------------------------------------------------------------------
# AXIS 3 — Focus diagnostic
# ---------------------------------------------------------------------------

@dataclass
class FocusDiagnosis:
    status: str                         # GOOD / MARGINAL / POOR
    best_led: int
    best_score: float
    worst_led: int
    worst_score: float
    median_score: float
    centre_led_score: float
    gradient_detected: bool             # large spatial variation → sample tilt?
    gradient_direction: str             # "top-bright", "left-bright", etc.
    adjustment_instructions: List[str]
    per_led_scores: Dict[int, float]


def diagnose_focus(
    measurements: List[LEDMeasurement],
) -> FocusDiagnosis:
    """
    Assess focus quality from Laplacian variance across the full stack.

    A uniform, high score means the sample is well-focused across the FOV.
    A systematic gradient (e.g. top LEDs in focus, bottom out of focus)
    indicates sample tilt and is flagged separately from pure defocus.
    """
    scored = {m.led_number: m.laplacian_score for m in measurements}
    scores = np.array(list(scored.values()))

    best_led   = max(scored, key=scored.get)
    worst_led  = min(scored, key=scored.get)
    median_sc  = float(np.median(scores))
    centre_sc  = scored.get(CENTRE_LED, 0.0)

    # Detect spatial gradient across the 5×5 grid
    # Compare top two rows vs bottom two rows, and left vs right
    top_leds    = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    bottom_leds = [16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
    left_leds   = [1, 6, 11, 16, 21, 2, 7, 12, 17, 22]
    right_leds  = [5, 10, 15, 20, 25, 4, 9, 14, 19, 24]

    def _grp_median(leds):
        vals = [scored[n] for n in leds if n in scored]
        return float(np.median(vals)) if vals else 0.0

    top_m    = _grp_median(top_leds)
    bottom_m = _grp_median(bottom_leds)
    left_m   = _grp_median(left_leds)
    right_m  = _grp_median(right_leds)

    tb_diff = abs(top_m - bottom_m)
    lr_diff = abs(left_m - right_m)
    gradient_threshold = 0.3 * median_sc   # 30% variation indicates tilt

    gradient_detected = (tb_diff > gradient_threshold or lr_diff > gradient_threshold)
    if gradient_detected:
        if tb_diff > lr_diff:
            grad_dir = "top-bright" if top_m > bottom_m else "bottom-bright"
        else:
            grad_dir = "left-bright" if left_m > right_m else "right-bright"
    else:
        grad_dir = "uniform"

    instructions: List[str] = []

    if centre_sc < _FOCUS_WARN:
        instructions.append(
            f"FOCUS FAIL — centre LED score {centre_sc:.1f}"
            f" (threshold {_FOCUS_PASS:.1f})."
            f"  Adjust Z height (objective or stage):"
        )
        instructions.append(
            "  Run: python val.py <session>  then repeat while adjusting Z"
        )
        instructions.append(
            "  Best focus is at maximum Laplacian score for LED 13"
        )
    elif centre_sc < _FOCUS_PASS:
        instructions.append(
            f"FOCUS MARGINAL — score {centre_sc:.1f}"
            f" (PASS threshold {_FOCUS_PASS:.1f}, WARN {_FOCUS_WARN:.1f})."
            f"  Fine-adjust Z for sharper images"
        )

    if gradient_detected:
        if "bright" in grad_dir:
            side_high = grad_dir.split("-")[0]
            side_low  = {"top": "bottom", "bottom": "top",
                         "left": "right", "right": "left"}[side_high]
            instructions.append(
                f"TILT DETECTED — focus score is higher on {side_high} than {side_low}."
                f"  Sample may be tilted or not flat-mounted."
                f"  Rotate/tilt the LED array or sample stage to equalise"
            )

    if centre_sc >= _FOCUS_PASS and not gradient_detected:
        instructions.append(
            f"Focus OK — centre score {centre_sc:.1f} (threshold {_FOCUS_PASS:.1f})"
        )

    # Blank slide warning: score near zero even at correct focus
    if median_sc < 1.0:
        instructions.append(
            "NOTE: Very low scores across all LEDs may indicate a blank slide"
            " or coverslip with no features — this is expected and not a"
            " focus problem.  Use a USAF 1951 target or patterned sample"
            " to verify focus."
        )

    if centre_sc >= _FOCUS_PASS:
        status = "GOOD"
    elif centre_sc >= _FOCUS_WARN:
        status = "MARGINAL"
    else:
        status = "POOR"

    return FocusDiagnosis(
        status               = status,
        best_led             = best_led,
        best_score           = round(scored[best_led], 2),
        worst_led            = worst_led,
        worst_score          = round(scored[worst_led], 2),
        median_score         = round(median_sc, 2),
        centre_led_score     = round(centre_sc, 2),
        gradient_detected    = gradient_detected,
        gradient_direction   = grad_dir,
        adjustment_instructions = instructions,
        per_led_scores       = {k: round(v, 2) for k, v in scored.items()},
    )


# ---------------------------------------------------------------------------
# Full diagnosis
# ---------------------------------------------------------------------------

@dataclass
class DiagnosisReport:
    session: str
    mode: str
    image_shape: List[int]
    leds_measured: int
    geometry: GeometryDiagnosis
    exposure: ExposureDiagnosis
    focus: FocusDiagnosis


def run_diagnosis(
    session_path: Path,
    cfg: Optional[SystemConfig] = None,
) -> DiagnosisReport:
    """
    Run all three diagnostic axes on an existing session directory.

    Args:
        session_path: Path to the session directory containing led_NN.png files.
        cfg:          Optional SystemConfig override.  Defaults to global config.

    Returns:
        DiagnosisReport with per-axis findings and adjustment instructions.
    """
    if cfg is None:
        cfg = _global_config

    if not session_path.exists():
        raise FileNotFoundError(f"Session not found: {session_path}")

    measurements, image_shape = measure_session(session_path)
    if len(measurements) < 10:
        raise ValueError(
            f"Only {len(measurements)} readable images — need at least 10"
        )

    # Try to read mode from status.json
    mode = cfg.mode
    status_file = session_path / "status.json"
    if status_file.exists():
        try:
            with open(status_file) as f:
                mode = json.load(f).get("mode", cfg.mode)
        except Exception:
            pass

    geom = diagnose_geometry(measurements, image_shape, cfg)
    expo = diagnose_exposure(measurements, cfg)
    focus = diagnose_focus(measurements)

    return DiagnosisReport(
        session       = session_path.name,
        mode          = mode,
        image_shape   = list(image_shape),
        leds_measured = len(measurements),
        geometry      = geom,
        exposure      = expo,
        focus         = focus,
    )


# ---------------------------------------------------------------------------
# Text report
# ---------------------------------------------------------------------------

def format_report(report: DiagnosisReport) -> str:
    """Render the DiagnosisReport as a readable plain-text summary."""
    g = report.geometry
    e = report.exposure
    f = report.focus

    WIDTH = 66

    def _section(title: str) -> str:
        pad = WIDTH - len(title) - 4
        return f"\n  ── {title} {'─' * pad}\n"

    def _status_badge(s: str) -> str:
        return {"GOOD": "[ GOOD ]", "MARGINAL": "[MARGINAL]", "POOR": "[ POOR ]"}.get(s, s)

    lines = [
        "=" * WIDTH,
        "  FPM POST-HOC DIAGNOSTIC REPORT",
        "=" * WIDTH,
        f"  Session   : {report.session}",
        f"  Mode      : {report.mode.upper()}",
        f"  Images    : {report.leds_measured} / 25",
        f"  Shape     : {report.image_shape[1]} × {report.image_shape[0]} px",
    ]

    # ── Geometry ────────────────────────────────────────────────────────
    lines.append(_section(f"1. GEOMETRY  {_status_badge(g.status)}"))
    if g.inferred_height_mm is not None:
        lines.append(
            f"  Inferred height  : {g.inferred_height_mm:.1f} mm"
            f"  (config: {g.config_height_mm:.1f} mm,"
            f"  Δ = {g.height_error_mm:+.1f} mm)"
        )
    else:
        lines.append(f"  Height fit       : not converged (scipy not available)")
    lines.append(
        f"  Array offset     : x={g.array_offset_x_mm:+.2f} mm,"
        f"  y={g.array_offset_y_mm:+.2f} mm  (at sample plane)"
    )
    lines.append(f"  LED 13 offset    : ({g.centre_led_offset_x_px:+.1f}, {g.centre_led_offset_y_px:+.1f}) px from image centre")
    if g.fit_converged:
        lines.append(f"  RMS residual     : {g.rms_residual_px:.1f} px  (max {g.max_residual_px:.1f} px on LED {g.worst_led})")
    lines.append("")
    for inst in g.adjustment_instructions:
        lines.append(f"  {inst}")

    # Per-LED residual grid
    lines.append("")
    lines.append("  Per-LED residuals (px after geometry fit):")
    lines.append("  ┌────┬────┬────┬────┬────┐")
    for row in range(5):
        cols = [g.per_led_residuals.get(row * 5 + col + 1, 0) for col in range(5)]
        lines.append("  │" + "│".join(f"{v:4.0f}" for v in cols) + "│")
    lines.append("  └────┴────┴────┴────┴────┘")

    # ── Exposure ─────────────────────────────────────────────────────────
    lines.append(_section(f"2. EXPOSURE  {_status_badge(e.status)}"))
    lines.append(f"  BF median  : {e.bf_mean_percent:.1f}% FSR  (target 30–65%)")
    lines.append(f"  DF median  : {e.df_mean_percent:.1f}% FSR  (target 15–50%)")
    lines.append(f"  BF current : {e.bf_current_us:,} us  ({e.bf_current_us/1e6:.1f} s)")
    lines.append(f"  DF current : {e.df_current_us:,} us  ({e.df_current_us/1e6:.1f} s)")
    if e.bf_recommended_us:
        lines.append(
            f"  BF → recommend : {e.bf_recommended_us:,} us"
            f"  ({e.bf_recommended_us/1e6:.1f} s,  {e.bf_scale_factor:.1f}×)"
        )
    if e.df_recommended_us:
        lines.append(
            f"  DF → recommend : {e.df_recommended_us:,} us"
            f"  ({e.df_recommended_us/1e6:.1f} s,  {e.df_scale_factor:.1f}×)"
        )
    lines.append("")
    for inst in e.adjustment_instructions:
        lines.append(f"  {inst}")

    # Per-LED exposure grid
    lines.append("")
    lines.append("  Per-LED mean intensity (% FSR):")
    lines.append("  ┌──────┬──────┬──────┬──────┬──────┐")
    for row in range(5):
        cols = [e.per_led_mean_percent.get(row * 5 + col + 1, 0) for col in range(5)]
        lines.append("  │" + "│".join(f"{v:5.1f}%" for v in cols) + "│")
    lines.append("  └──────┴──────┴──────┴──────┴──────┘")
    lines.append("  (* = BF LED)")

    # ── Focus ─────────────────────────────────────────────────────────────
    lines.append(_section(f"3. FOCUS     {_status_badge(f.status)}"))
    lines.append(f"  Centre LED score : {f.centre_led_score:.2f}  (PASS ≥ {_FOCUS_PASS:.1f},  WARN ≥ {_FOCUS_WARN:.1f})")
    lines.append(f"  Median score     : {f.median_score:.2f}")
    lines.append(f"  Best             : LED {f.best_led}  ({f.best_score:.2f})")
    lines.append(f"  Worst            : LED {f.worst_led}  ({f.worst_score:.2f})")
    lines.append(f"  Spatial gradient : {'YES — ' + f.gradient_direction if f.gradient_detected else 'none'}")
    lines.append("")
    for inst in f.adjustment_instructions:
        lines.append(f"  {inst}")

    # Per-LED focus grid
    lines.append("")
    lines.append("  Per-LED Laplacian score:")
    lines.append("  ┌───────┬───────┬───────┬───────┬───────┐")
    for row in range(5):
        cols = [f.per_led_scores.get(row * 5 + col + 1, 0) for col in range(5)]
        lines.append("  │" + "│".join(f"{v:6.2f} " for v in cols) + "│")
    lines.append("  └───────┴───────┴───────┴───────┴───────┘")

    # ── Summary ───────────────────────────────────────────────────────────
    lines.append(f"\n{'=' * WIDTH}")
    lines.append("  SUMMARY OF REQUIRED ACTIONS")
    lines.append("=" * WIDTH)
    has_action = False
    for axis_name, axis in [("Geometry", g), ("Exposure", e), ("Focus", f)]:
        if axis.status != "GOOD":
            for inst in axis.adjustment_instructions:
                if inst.startswith(("CONFIG:", "PHYSICAL:", "WIRING:", "WARNING:", "FOCUS")):
                    lines.append(f"  [{axis_name:8}]  {inst}")
                    has_action = True
    if not has_action:
        lines.append("  All axes GOOD — ready to acquire")
    lines.append("=" * WIDTH)

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Optional visualisation
# ---------------------------------------------------------------------------

def _plot_diagnosis(
    report: DiagnosisReport,
    measurements: List[LEDMeasurement],
    image_shape: Tuple[int, int],
    out_path: Path,
    cfg: SystemConfig,
):
    """
    3×2 matplotlib figure saved to out_path.

    [0,0] Geometry — centroid map with predicted overlay
    [0,1] Geometry — per-LED residual heat-map
    [1,0] Exposure — per-LED mean% bar chart
    [1,1] Exposure — BF vs DF annotated scatter
    [2,0] Focus    — per-LED Laplacian heat-map
    [2,1] Focus    — overall summary text panel
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.colors import Normalize
        from matplotlib.cm import ScalarMappable
    except ImportError:
        logger.warning("matplotlib not available — skipping plot")
        return

    g = report.geometry
    e = report.exposure
    f = report.focus

    fig, axes = plt.subplots(3, 2, figsize=(14, 18))
    fig.suptitle(
        f"FPM Diagnostic — {report.session}  [{report.mode.upper()}]",
        fontsize=14, fontweight="bold", y=0.98
    )

    h_px, w_px = image_shape
    ic_x, ic_y = w_px / 2.0, h_px / 2.0

    STATUS_COLOURS = {"GOOD": "#2ecc71", "MARGINAL": "#f39c12", "POOR": "#e74c3c"}

    # ── [0,0] Centroid map ───────────────────────────────────────────────
    ax = axes[0, 0]
    ax.set_title("Geometry: Centroid Map", fontsize=11)
    ax.set_facecolor("#1a1a2e")
    ax.set_xlim(0, w_px)
    ax.set_ylim(h_px, 0)
    ax.set_aspect("equal")
    ax.axhline(ic_y, color="white", lw=0.5, alpha=0.3)
    ax.axvline(ic_x, color="white", lw=0.5, alpha=0.3)

    # Predicted positions at inferred geometry
    if g.fit_converged and g.inferred_height_mm:
        cx_pred, cy_pred = _predicted_shifts_pixels(
            g.inferred_height_mm / 1000.0,
            g.array_offset_x_mm / 1000.0,
            g.array_offset_y_mm / 1000.0,
            cfg.optical.led_pitch_m,
            cfg.optical.wavelength_m,
            cfg.optical.pixel_size_m,
            image_shape,
        )
        ax.scatter(cx_pred, cy_pred, marker="+", color="#3498db",
                   s=80, zorder=3, label="predicted")

    valid = [m for m in measurements if m.centroid_ok]
    if valid:
        xs = [m.centroid_x for m in valid]
        ys = [m.centroid_y for m in valid]
        resids = [g.per_led_residuals.get(m.led_number, 0) for m in valid]
        norm = Normalize(vmin=0, vmax=max(resids) if resids else 1)
        colours = plt.cm.RdYlGn_r(norm(resids))
        ax.scatter(xs, ys, c=colours, s=60, zorder=4, label="measured")
        for m in valid:
            ax.annotate(str(m.led_number), (m.centroid_x, m.centroid_y),
                        fontsize=6, color="white", ha="center", va="center")

    ax.legend(fontsize=8, facecolor="#2d2d2d", labelcolor="white")
    ax.set_xlabel("X (px)", color="white", fontsize=9)
    ax.set_ylabel("Y (px)", color="white", fontsize=9)
    ax.tick_params(colors="white", labelsize=8)
    for sp in ax.spines.values():
        sp.set_color("#444")

    # ── [0,1] Residual heat-map ──────────────────────────────────────────
    ax = axes[0, 1]
    ax.set_title("Geometry: Residuals per LED (px)", fontsize=11)
    grid = np.zeros((5, 5))
    for n, r in g.per_led_residuals.items():
        row, col = divmod(n - 1, 5)
        grid[row, col] = r
    im = ax.imshow(grid, cmap="RdYlGn_r", vmin=0, vmax=max(60, grid.max()))
    fig.colorbar(im, ax=ax, label="Residual (px)")
    for row in range(5):
        for col in range(5):
            led_n = row * 5 + col + 1
            val   = g.per_led_residuals.get(led_n, 0)
            ax.text(col, row, f"{val:.0f}", ha="center", va="center",
                    fontsize=9, color="white")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel(f"RMS={g.rms_residual_px:.1f} px    Fit={'OK' if g.fit_converged else 'FAILED'}",
                  fontsize=9)

    # ── [1,0] Exposure bar chart ─────────────────────────────────────────
    ax = axes[1, 0]
    ax.set_title("Exposure: Mean Intensity per LED", fontsize=11)
    leds = sorted(e.per_led_mean_percent.keys())
    vals = [e.per_led_mean_percent[n] for n in leds]
    colours_exp = [
        "#2ecc71" if 20 <= v <= 70 else
        "#f39c12" if 8 <= v < 20 or 70 < v <= 80 else
        "#e74c3c"
        for v in vals
    ]
    bars = ax.bar(leds, vals, color=colours_exp, edgecolor="none")
    ax.axhspan(30, 65, alpha=0.15, color="#2ecc71", label="target zone")
    ax.axhline(20, color="#f39c12", ls="--", lw=1, alpha=0.7, label="warn low")
    ax.axhline(75, color="#e74c3c", ls="--", lw=1, alpha=0.7, label="fail high")
    ax.set_ylim(0, max(100, max(vals) * 1.1))
    ax.set_xlabel("LED index", fontsize=9)
    ax.set_ylabel("Mean intensity (% FSR)", fontsize=9)
    ax.legend(fontsize=8)
    ax.set_xticks(range(1, 26, 2))
    ax.tick_params(labelsize=8)

    # ── [1,1] BF vs DF scatter ───────────────────────────────────────────
    ax = axes[1, 1]
    ax.set_title("Exposure: BF vs DF Summary", fontsize=11)
    bf_vals = [e.per_led_mean_percent[n] for n in leds if n in BF_LEDS]
    df_vals = [e.per_led_mean_percent[n] for n in leds if n not in BF_LEDS]
    ax.boxplot([bf_vals, df_vals], labels=["BF (inner 3×3)", "DF (outer 16)"],
               patch_artist=True,
               boxprops=dict(facecolor="#3498db", alpha=0.6),
               medianprops=dict(color="white", lw=2))
    ax.axhspan(30, 65, alpha=0.15, color="#2ecc71", label="target zone")
    ax.axhline(20, color="#f39c12", ls="--", lw=1, alpha=0.7)
    ax.set_ylabel("Mean intensity (% FSR)", fontsize=9)
    ax.set_title(
        f"Exposure: BF={e.bf_mean_percent:.1f}%  DF={e.df_mean_percent:.1f}%  "
        + (f"→ BF×{e.bf_scale_factor:.1f}  DF×{e.df_scale_factor:.1f}" if e.bf_recommended_us else "OK"),
        fontsize=10
    )
    ax.tick_params(labelsize=9)
    ax.legend(fontsize=8)

    # ── [2,0] Focus heat-map ─────────────────────────────────────────────
    ax = axes[2, 0]
    ax.set_title("Focus: Laplacian Score per LED", fontsize=11)
    fgrid = np.zeros((5, 5))
    for n, s in f.per_led_scores.items():
        row, col = divmod(n - 1, 5)
        fgrid[row, col] = s
    fim = ax.imshow(fgrid, cmap="RdYlGn", vmin=0, vmax=max(_FOCUS_PASS * 3, fgrid.max()))
    fig.colorbar(fim, ax=ax, label="Laplacian variance")
    for row in range(5):
        for col in range(5):
            led_n = row * 5 + col + 1
            s     = f.per_led_scores.get(led_n, 0)
            ax.text(col, row, f"{s:.1f}", ha="center", va="center",
                    fontsize=8, color="white")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel(
        f"Centre={f.centre_led_score:.2f}  Median={f.median_score:.2f}"
        + (f"  Gradient: {f.gradient_direction}" if f.gradient_detected else ""),
        fontsize=9
    )

    # ── [2,1] Summary text panel ──────────────────────────────────────────
    ax = axes[2, 1]
    ax.axis("off")
    ax.set_facecolor("#1a1a2e")

    summary_lines = ["REQUIRED ACTIONS\n"]
    for axis_name, axis_diag in [("Geometry", g), ("Exposure", e), ("Focus", f)]:
        colour = STATUS_COLOURS.get(axis_diag.status, "white")
        summary_lines.append(f"[{axis_name}]  {axis_diag.status}")
        for inst in axis_diag.adjustment_instructions:
            if len(inst) < 70:
                summary_lines.append(f"  {inst}")
            else:
                # word-wrap at 68 chars
                words = inst.split()
                line  = "  "
                for w in words:
                    if len(line) + len(w) > 68:
                        summary_lines.append(line)
                        line = "    " + w + " "
                    else:
                        line += w + " "
                summary_lines.append(line.rstrip())
        summary_lines.append("")

    summary_text = "\n".join(summary_lines)
    ax.text(
        0.02, 0.97, summary_text,
        transform=ax.transAxes,
        fontsize=8, va="top", ha="left",
        family="monospace",
        color="white",
        bbox=dict(facecolor="#1a1a2e", edgecolor="#444", boxstyle="round,pad=0.5")
    )

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(str(out_path), dpi=130, bbox_inches="tight", facecolor="#111")
    plt.close(fig)
    logger.info(f"Diagnostic plot saved: {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    parser = argparse.ArgumentParser(
        description=(
            "FPM post-hoc diagnostic — reads an existing 25-image session "
            "and outputs specific quantitative adjustment instructions."
        )
    )
    parser.add_argument(
        "session",
        type=Path,
        help="Path to session directory containing led_01.png … led_25.png"
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip generating diagnosis_plot.png"
    )
    parser.add_argument(
        "--json-only",
        action="store_true",
        help="Print JSON to stdout and exit (no text report, no plot)"
    )
    parser.add_argument(
        "--height",
        type=float,
        default=None,
        metavar="MM",
        help="Override led_height_mm for geometry fit starting point"
    )
    args = parser.parse_args()

    session_path = args.session.resolve()
    if not session_path.exists():
        print(f"ERROR: Session not found: {session_path}", file=sys.stderr)
        sys.exit(1)

    cfg = _global_config
    if args.height:
        # Patch config height without modifying global singleton
        import copy
        cfg = copy.deepcopy(_global_config)
        object.__setattr__(cfg.optical, "led_height_mm", float(args.height))

    report = run_diagnosis(session_path, cfg=cfg)

    if args.json_only:
        print(json.dumps(asdict(report), indent=2, default=str))
        sys.exit(0)

    text = format_report(report)
    print(text)

    # Write outputs
    (session_path / "diagnosis_summary.txt").write_text(text, encoding="utf-8")
    print(f"\n  Written: {session_path / 'diagnosis_summary.txt'}")

    report_dict = asdict(report)
    (session_path / "diagnosis_report.json").write_text(
        json.dumps(report_dict, indent=2, default=str), encoding="utf-8"
    )
    print(f"  Written: {session_path / 'diagnosis_report.json'}")

    if not args.no_plot:
        # Re-measure to pass raw measurements to plotter
        measurements, image_shape = measure_session(session_path)
        plot_path = session_path / "diagnosis_plot.png"
        _plot_diagnosis(report, measurements, image_shape, plot_path, cfg)
        print(f"  Written: {plot_path}")

    sys.exit(0 if all(
        ax.status != "POOR"
        for ax in [report.geometry, report.exposure, report.focus]
    ) else 1)


if __name__ == "__main__":
    main()