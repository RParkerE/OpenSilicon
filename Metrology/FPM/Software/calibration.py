"""
FPM Calibration Module — Raspberry Pi Side (headless)

Runs entirely on the Pi. Pi-side dependencies only:
    config.py   — optical + acquisition parameters
    client.py   — ESP32Controller, CameraInterface
    cv2, numpy, requests, stdlib

NO imports from: server.py, reconstruction.py, utils.py

Three checks:
    1. Exposure  — image not blown-out or too dark; auto-tunes exposure_us
    2. Alignment — LED centroid is centred on the sensor optical axis
    3. Focus     — substrate is in sharp focus (normalised Laplacian variance)

Headless output:
    Results are written to /tmp/fpm_calibration/ on the Pi AND pushed to the
    FastAPI server via the existing /upload/{session_id} endpoint.
    No new server endpoints required.

    Pull results from your laptop:
        # Plain-text summary (readable in terminal)
        curl http://<pi-ip>:8000 ...
        # Or directly from the Pi over SSH:
        ssh pi@<pi-ip> cat /tmp/fpm_calibration/calibration_summary.txt
        ssh pi@<pi-ip> cat /tmp/fpm_calibration/calibration_report.json
        # Pull annotated alignment image
        scp pi@<pi-ip>:/tmp/fpm_calibration/alignment_annotated.png .

Usage (standalone over SSH):
    python calibration.py                                       # full suite
    python calibration.py --mode exposure                       # single check
    python calibration.py --mode focus --no-upload              # local only
    python calibration.py --server http://192.168.1.78:8000     # custom server

Usage (from client.py before capture_sequence):
    from calibration import CalibrationSuite, CheckStatus
    suite  = CalibrationSuite(esp32, camera, server_url=args.server)
    result = suite.run_all(exposure_us=bf_exp, tune_exposure=True)
    if not result.passed:
        sys.exit(1)
    if result.exposure.recommended_exposure_us:
        bf_exp = result.exposure.recommended_exposure_us
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import Optional

import threading

import cv2
import numpy as np
import requests

from imgload import load_uv_image, channel_report

from config import config

logger = logging.getLogger("FPMCalibration")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Reserved session name on the server for calibration artefacts
_CAL_SESSION = "_calibration_latest"

# Default local work directory on the Pi
_LOCAL_CAL_DIR = Path("/tmp/fpm_calibration")


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class CheckStatus(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"


@dataclass
class ExposureReport:
    status: CheckStatus
    led_index: int
    exposure_us: int
    mean_percent: float           # mean pixel as % of full scale
    saturation_percent: float     # % pixels >= 95 % FSR
    black_clip_percent: float     # % pixels <= 1 % FSR
    recommended_exposure_us: Optional[int]
    message: str


@dataclass
class AlignmentReport:
    status: CheckStatus
    centroid_x_px: float
    centroid_y_px: float
    sensor_center_x: float
    sensor_center_y: float
    offset_x_px: float
    offset_y_px: float
    offset_x_percent: float
    offset_y_percent: float
    message: str


@dataclass
class FocusReport:
    status: CheckStatus
    laplacian_score: float
    brenner_score: float
    focus_direction_hint: str   # "in_focus" | "move_closer" | "move_further" | "unknown"
    message: str


@dataclass
class CalibrationResult:
    timestamp: str
    illumination_mode: str
    passed: bool
    exposure: ExposureReport
    alignment: AlignmentReport
    focus: FocusReport
    warnings: list = field(default_factory=list)

    def summary(self) -> str:
        overall = "PASSED" if self.passed else "FAILED"
        lines = [
            "=" * 62,
            "  FPM CALIBRATION REPORT",
            "=" * 62,
            f"  Time      : {self.timestamp}",
            f"  Mode      : {self.illumination_mode}",
            f"  Overall   : {overall}",
            "",
            f"  Exposure  [{self.exposure.status.value:^7}]  {self.exposure.message}",
            f"  Alignment [{self.alignment.status.value:^7}]  {self.alignment.message}",
            f"  Focus     [{self.focus.status.value:^7}]  {self.focus.message}",
        ]
        if self.warnings:
            lines.append("")
            lines.append("  Warnings:")
            for w in self.warnings:
                lines.append(f"    !  {w}")
        if self.exposure.recommended_exposure_us:
            lines.append("")
            lines.append(
                f"  Recommended exposure: "
                f"{self.exposure.recommended_exposure_us} us"
            )
        lines.append("=" * 62)
        return "\n".join(lines)

    def to_json(self) -> str:
        d = asdict(self)
        d["exposure"]["status"]  = self.exposure.status.value
        d["alignment"]["status"] = self.alignment.status.value
        d["focus"]["status"]     = self.focus.status.value
        return json.dumps(d, indent=2)


# ---------------------------------------------------------------------------
# Tunable thresholds
# ---------------------------------------------------------------------------

class _T:
    # ---- Exposure (IMX219 via rpicam-still, 8-bit PNG output) ----
    MEAN_FAIL_LOW:   float = 20.0    # % FSR — below here: FAIL (too dark)
    MEAN_WARN_LOW:   float = 30.0    # % FSR — below here: WARN
    MEAN_WARN_HIGH:  float = 65.0    # % FSR — above here: WARN
    MEAN_FAIL_HIGH:  float = 75.0    # % FSR — above here: FAIL (too bright)
    SAT_WARN:        float = 0.1     # % pixels >= 95 % FSR -> WARN
    SAT_FAIL:        float = 0.5     # % pixels >= 95 % FSR -> FAIL
    CLIP_FAIL:       float = 5.0     # % pixels <= 1 % FSR  -> FAIL

    # ---- Auto-tune binary search ----
    TUNE_MAX_ITER:   int   = 8
    TUNE_MAX_RATIO:  float = 4.0     # max scale factor per step
    EXPOSURE_MIN_US: int   = 100_000
    EXPOSURE_MAX_US: int   = 120_000_000
    ESP32_MAX_MS:    int   = 180_000   # firmware cap from ESP32 sketch

    # ---- Alignment (centroid offset from sensor centre) ----
    ALIGN_WARN_PCT:  float = 3.0     # % of sensor half-width
    ALIGN_FAIL_PCT:  float = 5.0

    # ---- Focus (normalised Laplacian variance / mean(I)) ----
    # Tune by running with in-focus and defocused samples on your hardware.
    # Method: defocus deliberately -> note score (FAIL floor)
    #         refocus by eye       -> note score (PASS baseline)
    FOCUS_FAIL:      float = 0.15
    FOCUS_WARN:      float = 0.40


# ---------------------------------------------------------------------------
# Calibration engine
# ---------------------------------------------------------------------------

class CalibrationSuite:
    """
    Runs exposure, alignment, and focus checks using Pi-local hardware only.

    Args:
        esp32:      Connected ESP32Controller instance
        camera:     CameraInterface instance
        server_url: FastAPI server URL for pushing results (None = local only)
        work_dir:   Local directory on the Pi for output files
    """

    CENTRE_LED: int = 13   # 1-indexed, row-major 5x5 grid

    def __init__(
        self,
        esp32,
        camera,
        server_url: Optional[str] = None,
        work_dir: Path = _LOCAL_CAL_DIR
    ):
        self.esp32      = esp32
        self.camera     = camera
        self.server_url = server_url
        self.work_dir   = work_dir
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self._fsr       = 255.0   # 8-bit full scale

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run_all(
        self,
        exposure_us: Optional[int] = None,
        tune_exposure: bool = True
    ) -> CalibrationResult:
        """
        Run all three checks: exposure -> alignment -> focus.

        If tune_exposure is True and exposure fails/warns, runs binary search
        to find a usable exposure before the alignment and focus checks, so
        those frames are captured at a meaningful brightness level.
        """
        exp_us = exposure_us or config.acquisition.bf_exposure_us

        # 1. Exposure
        exp_report = self.check_exposure(exposure_us=exp_us)

        if tune_exposure and exp_report.status != CheckStatus.PASS:
            logger.info("Exposure out of range — running auto-tune")
            tuned = self._auto_tune_exposure(exp_us)
            if tuned:
                exp_report = self.check_exposure(exposure_us=tuned)
                exp_report.recommended_exposure_us = tuned

        best_us = exp_report.recommended_exposure_us or exp_us

        # 2. Alignment
        align_report = self.check_alignment(exposure_us=best_us)

        # 3. Focus
        focus_report = self.check_focus(exposure_us=best_us)

        # Aggregate pass/fail.
        # Alignment failure is excluded from the hard-fail gate: it indicates
        # physical LED-array misalignment that cannot be fixed in software.
        # It still shows as FAIL in the report and should be corrected, but it
        # should not abort an acquisition run — captures are still usable.
        hard_statuses = [exp_report.status, focus_report.status]
        passed        = CheckStatus.FAIL not in hard_statuses

        warnings = []
        if exp_report.status == CheckStatus.WARN:
            rec = exp_report.recommended_exposure_us
            warnings.append(
                f"Exposure marginal ({exp_report.mean_percent:.1f}% FSR)"
                + (f" -- try {rec} us" if rec else "")
            )
        if align_report.status == CheckStatus.WARN:
            warnings.append(
                f"Centroid offset ({align_report.offset_x_percent:.1f}%, "
                f"{align_report.offset_y_percent:.1f}%) -- nudge LED array"
            )
        if focus_report.status == CheckStatus.WARN:
            warnings.append(
                f"Focus soft (score={focus_report.laplacian_score:.3f}) -- "
                f"{focus_report.focus_direction_hint}"
            )

        result = CalibrationResult(
            timestamp=time.strftime("%Y-%m-%d %H:%M:%S"),
            illumination_mode=config.optical.illumination_mode.value,
            passed=passed,
            exposure=exp_report,
            alignment=align_report,
            focus=focus_report,
            warnings=warnings
        )

        # Always print to stdout — visible over SSH
        print(result.summary())

        self._save_local(result)

        if self.server_url:
            self._push_to_server()

        return result

    # ------------------------------------------------------------------
    # 1. Exposure Check
    # ------------------------------------------------------------------

    def check_exposure(self, exposure_us: Optional[int] = None) -> ExposureReport:
        """
        Capture one frame with the centre LED and evaluate pixel statistics.

        Metrics:
            mean_percent      — mean pixel value as % of 8-bit full scale
            saturation_percent — % pixels >= 95% FSR (hard clipping)
            black_clip_percent — % pixels <= 1% FSR (underexposed regions)

        Both extremes make the image unusable for FPM:
            Too bright: amplitude constraint sqrt(I) is clipped, fringes lost
            Too dark:   shot noise dominates, phase retrieval diverges

        Auto-recommends a corrected exposure via linear scaling toward the
        centre of the acceptable window (47.5% FSR).
        """
        exp_us = exposure_us or config.acquisition.bf_exposure_us
        logger.info(f"Exposure check -- {exp_us} us")

        frame = self._capture(self.CENTRE_LED, exp_us, tag="exposure")
        if frame is None:
            return ExposureReport(
                status=CheckStatus.FAIL,
                led_index=self.CENTRE_LED,
                exposure_us=exp_us,
                mean_percent=0.0,
                saturation_percent=0.0,
                black_clip_percent=0.0,
                recommended_exposure_us=None,
                message="Capture failed -- check camera connection and LED"
            )

        mean_val = float(np.mean(frame))
        mean_pct = mean_val / self._fsr * 100.0
        sat_pct  = float(np.sum(frame >= 0.95 * self._fsr)) / frame.size * 100.0
        clip_pct = float(np.sum(frame <= 0.01 * self._fsr)) / frame.size * 100.0

        # Scale toward target centre of the warn band (47.5% FSR)
        target_pct = (_T.MEAN_WARN_LOW + _T.MEAN_WARN_HIGH) / 2.0
        scale      = (target_pct / mean_pct) if mean_pct > 0 else 1.0
        scale      = float(np.clip(scale, 1.0 / _T.TUNE_MAX_RATIO, _T.TUNE_MAX_RATIO))
        rec_us     = int(np.clip(exp_us * scale, _T.EXPOSURE_MIN_US, _T.EXPOSURE_MAX_US))
        # Suppress recommendation if change is trivial (< 5%)
        rec_us = rec_us if abs(rec_us - exp_us) / exp_us > 0.05 else None

        if sat_pct > _T.SAT_FAIL:
            status  = CheckStatus.FAIL
            message = (
                f"OVEREXPOSED -- {sat_pct:.2f}% pixels saturated. "
                f"Reduce to ~{rec_us} us"
            )
        elif mean_pct > _T.MEAN_FAIL_HIGH:
            status  = CheckStatus.FAIL
            message = (
                f"TOO BRIGHT -- mean={mean_pct:.1f}% FSR. "
                f"Reduce to ~{rec_us} us"
            )
        elif mean_pct > _T.MEAN_WARN_HIGH or sat_pct > _T.SAT_WARN:
            status  = CheckStatus.WARN
            message = (
                f"Bright ({mean_pct:.1f}% FSR, sat={sat_pct:.2f}%). "
                f"Consider {rec_us} us"
            )
        elif mean_pct < _T.MEAN_FAIL_LOW or clip_pct > _T.CLIP_FAIL:
            status  = CheckStatus.FAIL
            message = (
                f"UNDEREXPOSED -- mean={mean_pct:.1f}% FSR, "
                f"black-clip={clip_pct:.1f}%. Increase to ~{rec_us} us"
            )
        elif mean_pct < _T.MEAN_WARN_LOW:
            status  = CheckStatus.WARN
            message = f"Dark ({mean_pct:.1f}% FSR). Consider {rec_us} us"
        else:
            status  = CheckStatus.PASS
            rec_us  = None
            message = f"OK -- mean={mean_pct:.1f}% FSR, sat={sat_pct:.2f}%"

        logger.info(f"  Exposure: {status.value} -- {message}")
        return ExposureReport(
            status=status,
            led_index=self.CENTRE_LED,
            exposure_us=exp_us,
            mean_percent=round(mean_pct, 2),
            saturation_percent=round(sat_pct, 3),
            black_clip_percent=round(clip_pct, 2),
            recommended_exposure_us=rec_us,
            message=message
        )

    def _auto_tune_exposure(self, initial_us: int) -> Optional[int]:
        """
        Geometric (log-space) binary search for an exposure that lands
        in the WARN_LOW-WARN_HIGH band. Runs at most _T.TUNE_MAX_ITER captures.

        Uses geometric mean (sqrt(lo * hi)) rather than arithmetic because
        camera exposure response is multiplicative -- equal steps in log space
        produce equal steps in brightness.
        """
        lo      = float(_T.EXPOSURE_MIN_US)
        hi      = float(_T.EXPOSURE_MAX_US)
        best_us = initial_us
        tgt_lo  = _T.MEAN_WARN_LOW  / 100.0 * self._fsr
        tgt_hi  = _T.MEAN_WARN_HIGH / 100.0 * self._fsr

        for i in range(_T.TUNE_MAX_ITER):
            frame = self._capture(self.CENTRE_LED, best_us, tag=f"tune_{i}")
            if frame is None:
                logger.warning(f"  Auto-tune capture failed at iteration {i}")
                return None

            mean_val = float(np.mean(frame))
            logger.info(
                f"  Tune [{i+1}/{_T.TUNE_MAX_ITER}]: "
                f"{best_us} us -> mean={mean_val:.1f} "
                f"({mean_val / self._fsr * 100:.1f}% FSR)"
            )

            if tgt_lo <= mean_val <= tgt_hi:
                logger.info(f"  Converged at {best_us} us")
                return best_us

            if mean_val < tgt_lo:
                lo = float(best_us)
            else:
                hi = float(best_us)

            best_us = int(np.clip(
                np.sqrt(lo * hi),
                _T.EXPOSURE_MIN_US,
                _T.EXPOSURE_MAX_US
            ))

        logger.warning(
            f"  Auto-tune did not fully converge after {_T.TUNE_MAX_ITER} "
            f"iterations -- best attempt: {best_us} us"
        )
        return best_us

    # ------------------------------------------------------------------
    # 2. Alignment Check
    # ------------------------------------------------------------------

    def check_alignment(self, exposure_us: Optional[int] = None) -> AlignmentReport:
        """
        Fire the centre LED and compute the intensity-weighted centroid of the
        bright-field spot. Compare to the geometric centre of the sensor.

        Why it matters:
            A misaligned centroid means the LED array's optical axis is offset
            from the camera axis. This applies a constant DC bias to ALL 25
            k-space shifts, translating the entire synthetic aperture in Fourier
            space and corrupting the phase reconstruction.

        Physical correction:
            Translate (do not tilt) the LED array in x/y until the centroid
            coincides with the sensor centre. The annotated PNG saved locally
            shows both positions with directional arrows.

        Centroid method:
            Threshold at 50% of frame peak to isolate the bright-field spot,
            then compute intensity-weighted centroid of the thresholded region.
            Robust to background gradients and dark-field spill.
        """
        exp_us = exposure_us or config.acquisition.bf_exposure_us
        logger.info(f"Alignment check -- {exp_us} us")

        frame = self._capture(self.CENTRE_LED, exp_us, tag="alignment")
        if frame is None:
            return AlignmentReport(
                status=CheckStatus.FAIL,
                centroid_x_px=0, centroid_y_px=0,
                sensor_center_x=0, sensor_center_y=0,
                offset_x_px=0, offset_y_px=0,
                offset_x_percent=0, offset_y_percent=0,
                message="Capture failed"
            )

        h, w  = frame.shape
        cx, cy = w / 2.0, h / 2.0

        threshold    = 0.50 * float(frame.max())
        weighted_img = np.where(frame >= threshold, frame, 0.0).astype(np.float64)
        total        = weighted_img.sum()

        if total < 1.0:
            return AlignmentReport(
                status=CheckStatus.FAIL,
                centroid_x_px=cx, centroid_y_px=cy,
                sensor_center_x=cx, sensor_center_y=cy,
                offset_x_px=0, offset_y_px=0,
                offset_x_percent=0, offset_y_percent=0,
                message=(
                    "No bright region detected -- is LED firing? "
                    "Is the sample blocking the beam?"
                )
            )

        ys, xs  = np.mgrid[0:h, 0:w].astype(np.float64)
        centx   = float((weighted_img * xs).sum() / total)
        centy   = float((weighted_img * ys).sum() / total)
        off_x   = centx - cx
        off_y   = centy - cy
        off_xp  = abs(off_x) / (w / 2.0) * 100.0
        off_yp  = abs(off_y) / (h / 2.0) * 100.0
        worst   = max(off_xp, off_yp)

        # Physical offset at sensor plane (native pixel = 1.12 um, not effective)
        dx_um = off_x * config.optical.pixel_size_um
        dy_um = off_y * config.optical.pixel_size_um

        self._annotate_alignment(frame, centx, centy, cx, cy)

        if worst > _T.ALIGN_FAIL_PCT:
            dir_x = "left" if off_x > 0 else "right"
            dir_y = "up"   if off_y > 0 else "down"
            status  = CheckStatus.FAIL
            message = (
                f"MISALIGNED -- centroid ({centx:.0f}, {centy:.0f}) px, "
                f"offset ({off_x:+.0f}, {off_y:+.0f}) px "
                f"= ({dx_um:+.0f}, {dy_um:+.0f}) um at sensor. "
                f"Move LED array {dir_x} and {dir_y}"
            )
        elif worst > _T.ALIGN_WARN_PCT:
            status  = CheckStatus.WARN
            message = (
                f"Slight offset ({off_xp:.1f}%, {off_yp:.1f}%) -- "
                f"centroid ({centx:.0f}, {centy:.0f}) px"
            )
        else:
            status  = CheckStatus.PASS
            message = (
                f"OK -- centroid ({centx:.0f}, {centy:.0f}) px, "
                f"offset ({off_xp:.1f}%, {off_yp:.1f}%)"
            )

        logger.info(f"  Alignment: {status.value} -- {message}")
        return AlignmentReport(
            status=status,
            centroid_x_px=round(centx, 1),
            centroid_y_px=round(centy, 1),
            sensor_center_x=round(cx, 1),
            sensor_center_y=round(cy, 1),
            offset_x_px=round(off_x, 1),
            offset_y_px=round(off_y, 1),
            offset_x_percent=round(off_xp, 2),
            offset_y_percent=round(off_yp, 2),
            message=message
        )

    def _annotate_alignment(self, frame, centx, centy, cx, cy):
        """
        Save a side-by-side PNG to work_dir:
            Left  -- raw calibration frame (8-bit normalised)
            Right -- annotated:
                       Red crosshair   = detected centroid
                       Green crosshair = sensor target centre
                       Yellow circle   = WARN tolerance radius
                       Red circle      = FAIL tolerance radius

        This file is pushed to the server so it can be pulled with scp/curl
        on any machine that can reach the server or the Pi.
        """
        try:
            norm     = (frame / (frame.max() + 1e-8) * 255).astype(np.uint8)
            raw_bgr  = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
            ann_bgr  = raw_bgr.copy()

            cv2.drawMarker(ann_bgr, (int(centx), int(centy)),
                           (0, 0, 255), cv2.MARKER_CROSS, 60, 2)     # red
            cv2.drawMarker(ann_bgr, (int(cx), int(cy)),
                           (0, 255, 0), cv2.MARKER_CROSS, 60, 2)     # green

            warn_r = int(cx * _T.ALIGN_WARN_PCT / 100.0)
            fail_r = int(cx * _T.ALIGN_FAIL_PCT / 100.0)
            cv2.circle(ann_bgr, (int(cx), int(cy)), warn_r, (0, 255, 255), 1)  # yellow
            cv2.circle(ann_bgr, (int(cx), int(cy)), fail_r, (0, 0, 255),   1)  # red

            cv2.putText(
                ann_bgr, "RED=centroid  GREEN=target",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
            )

            # Scale to 1024px wide each side for manageable file size
            tw       = 1024
            scale    = tw / frame.shape[1]
            new_size = (tw, int(frame.shape[0] * scale))
            combined = np.hstack([
                cv2.resize(raw_bgr, new_size),
                cv2.resize(ann_bgr, new_size)
            ])

            out = self.work_dir / "alignment_annotated.png"
            cv2.imwrite(str(out), combined)
            logger.info(f"  Alignment image: {out}")
        except Exception as exc:
            logger.warning(f"  Could not save alignment image: {exc}")

    # ------------------------------------------------------------------
    # 3. Focus Check
    # ------------------------------------------------------------------

    def check_focus(self, exposure_us: Optional[int] = None) -> FocusReport:
        """
        Evaluate sharpness using the normalised Laplacian variance metric.

            F = Var(Laplacian(I)) / (Mean(I) + eps)

        High F = sharp edges = in focus.
        Normalisation by mean(I) prevents bright frames from inflating the score.

        Direction hint:
            Two bracketed frames at +-15% exposure are captured as a proxy for
            the focus curve slope (Laplacian is monotonic with contrast near
            best focus). This gives a coarse move direction without a Z-motor.

        Tuning thresholds (_T.FOCUS_FAIL, _T.FOCUS_WARN):
            1. Deliberately defocus by ~2 mm -> run check -> note score (FAIL floor)
            2. Refocus by eye -> run check -> note score (PASS baseline)
            3. Update _T.FOCUS_FAIL and _T.FOCUS_WARN to sit between those values
        """
        exp_us = exposure_us or config.acquisition.bf_exposure_us
        logger.info(f"Focus check -- {exp_us} us")

        frame = self._capture(self.CENTRE_LED, exp_us, tag="focus")
        if frame is None:
            return FocusReport(
                status=CheckStatus.FAIL,
                laplacian_score=0.0,
                brenner_score=0.0,
                focus_direction_hint="unknown",
                message="Capture failed"
            )

        lap   = self._laplacian_score(frame)
        bren  = self._brenner_score(frame)
        hint  = self._focus_direction(exp_us, lap)

        if lap < _T.FOCUS_FAIL:
            status  = CheckStatus.FAIL
            message = (
                f"OUT OF FOCUS -- score={lap:.4f} (threshold={_T.FOCUS_FAIL}). "
                f"Adjust Z: {hint}"
            )
        elif lap < _T.FOCUS_WARN:
            status  = CheckStatus.WARN
            message = f"Soft focus -- score={lap:.4f}. Fine-adjust Z. Hint: {hint}"
        else:
            status  = CheckStatus.PASS
            message = f"In focus -- score={lap:.4f}"

        logger.info(f"  Focus: {status.value} -- {message}")
        return FocusReport(
            status=status,
            laplacian_score=round(lap, 5),
            brenner_score=round(bren, 2),
            focus_direction_hint=hint,
            message=message
        )

    def _laplacian_score(self, frame) -> float:
        # Convert to float32 before Laplacian — CV_16U→CV_64F unsupported in OCV 4.13
        frame_f = frame.astype(np.float32)
        lap = cv2.Laplacian(frame_f, cv2.CV_32F)
        return float(lap.var())

    def _brenner_score(self, frame: np.ndarray) -> float:
        diff = frame[:, 2:].astype(np.float64) - frame[:, :-2].astype(np.float64)
        return float(np.sum(diff ** 2)) / frame.size

    def _focus_direction(self, exposure_us: int, current_score: float) -> str:
        """
        Bracket +-15% exposure and compare Laplacian scores as a proxy for
        focus curve slope direction.

        Dead-band: if the two scores differ by less than 2% of current_score
        the system is considered in focus (slope is flat at the peak).
        """
        delta  = 0.15
        us_lo  = int(np.clip(exposure_us * (1 - delta),
                              _T.EXPOSURE_MIN_US, _T.EXPOSURE_MAX_US))
        us_hi  = int(np.clip(exposure_us * (1 + delta),
                              _T.EXPOSURE_MIN_US, _T.EXPOSURE_MAX_US))

        f_lo = self._capture(self.CENTRE_LED, us_lo, tag="focus_lo")
        f_hi = self._capture(self.CENTRE_LED, us_hi, tag="focus_hi")

        if f_lo is None or f_hi is None:
            return "unknown"

        s_lo = self._laplacian_score(f_lo)
        s_hi = self._laplacian_score(f_hi)

        if abs(s_lo - s_hi) < 0.02 * (current_score + 1e-8):
            return "in_focus"
        elif s_lo > s_hi:
            return "move_closer (reduce working distance)"
        else:
            return "move_further (increase working distance)"

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    # Seconds to wait after starting rpicam-still before firing the LED.
    # rpicam-still takes ~2-3s to open the sensor pipeline before exposure begins.
    # Firing the LED before this point means it extinguishes before any photons
    # are collected, producing black frames.
    _CAMERA_STARTUP_S: float = 2.5

    def _capture(
        self,
        led_index: int,
        exposure_us: int,
        tag: str = "cal"
    ) -> Optional[np.ndarray]:
        """
        Start camera, wait for sensor pipeline to open, then fire the LED so
        the pulse overlaps with the actual exposure window.

        rpicam-still takes ~2-3 s to initialise before the sensor starts
        collecting photons. Firing the LED before that point produces black
        frames regardless of pulse duration. This method avoids that by
        launching the capture in a background thread, sleeping for
        _CAMERA_STARTUP_S, then sending the trigger.

        Returns float32 grayscale ndarray or None on failure.
        """
        out_path = self.work_dir / f"cal_{tag}_led{led_index:02d}.png"

        # Pulse long enough to span camera startup + full exposure + margin
        pulse_ms = int(self._CAMERA_STARTUP_S * 1000) + 600 + (exposure_us // 1000) + 500
        pulse_ms = min(pulse_ms, _T.ESP32_MAX_MS)

        capture_ok = [False]

        def _do_capture():
            capture_ok[0] = self.camera.capture(
                out_path,
                exposure_us=exposure_us,
                gain=config.acquisition.analog_gain,
                timeout_s=config.acquisition.capture_timeout_s
            )

        cam_thread = threading.Thread(target=_do_capture, daemon=True)
        cam_thread.start()

        # Wait for rpicam-still to open the sensor before firing the LED
        time.sleep(self._CAMERA_STARTUP_S)

        if not self.esp32.send_trigger(led_index, pulse_ms):
            logger.error(f"  ESP32 trigger failed for LED {led_index}")
            cam_thread.join(timeout=config.acquisition.capture_timeout_s + 5)
            return None

        cam_thread.join(timeout=config.acquisition.capture_timeout_s + 5)

        if not capture_ok[0] or not out_path.exists():
            return None

        img = load_uv_image(out_path)
        if img is None:
            logger.error(f"  load_uv_image returned None for {out_path}")
            return None

        logger.debug(f"  {out_path.name}: {channel_report(out_path)}")

        return img.astype(np.float32)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _save_local(self, result: CalibrationResult):
        """
        Write calibration_report.json and calibration_summary.txt to work_dir.

        Readable directly over SSH:
            cat /tmp/fpm_calibration/calibration_summary.txt
            cat /tmp/fpm_calibration/calibration_report.json
        """
        (self.work_dir / "calibration_report.json").write_text(result.to_json())
        (self.work_dir / "calibration_summary.txt").write_text(result.summary())
        logger.info(f"  Local output: {self.work_dir}")

    def _push_to_server(self):
        """
        Upload all calibration artefacts to the server using the existing
        POST /upload/{session_id} endpoint.

        Uses the reserved session name '_calibration_latest' so the results
        always overwrite the previous run and can be found at a known URL.
        No new server endpoints are required.

        Files pushed (whichever exist in work_dir):
            calibration_report.json
            calibration_summary.txt
            alignment_annotated.png
            cal_exposure_led13.png
            cal_focus_led13.png

        Pull from your laptop once uploaded:
            # JSON report (via server /sessions/{id}/status or direct file serve)
            scp pi@<pi-ip>:/tmp/fpm_calibration/calibration_summary.txt .
            # Or if server exposes static files:
            curl http://<server>:8000/sessions/_calibration_latest/status
        """
        upload_url = f"{self.server_url}/upload/{_CAL_SESSION}"

        candidates = [
            self.work_dir / "calibration_report.json",
            self.work_dir / "calibration_summary.txt",
            self.work_dir / "alignment_annotated.png",
            self.work_dir / f"cal_exposure_led{self.CENTRE_LED:02d}.png",
            self.work_dir / f"cal_focus_led{self.CENTRE_LED:02d}.png",
        ]

        files   = []
        handles = []
        for path in candidates:
            if path.exists():
                fh = open(path, "rb")
                handles.append(fh)
                files.append(("files", (path.name, fh, "application/octet-stream")))

        if not files:
            logger.warning("  No calibration files found to push")
            return

        try:
            resp = requests.post(upload_url, files=files, timeout=30)
            resp.raise_for_status()
            logger.info(f"  Pushed {len(files)} files to {upload_url}")
        except requests.exceptions.RequestException as exc:
            logger.warning(f"  Could not push to server: {exc}")
            logger.warning("  Results are still available locally via SSH")
        finally:
            for fh in handles:
                fh.close()


# ---------------------------------------------------------------------------
# CLI — runs directly on the Pi over SSH
# ---------------------------------------------------------------------------

def main():
    import argparse
    import sys

    # Lazy import avoids circular dependency if client.py imports calibration.py
    from client import ESP32Controller, CameraInterface

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    parser = argparse.ArgumentParser(
        description="FPM Calibration -- runs on Raspberry Pi, no display required"
    )
    parser.add_argument(
        "--mode",
        choices=["all", "exposure", "alignment", "focus"],
        default="all",
        help="Which check to run (default: all)"
    )
    parser.add_argument(
        "--server",
        type=str,
        default="http://192.168.1.78:8000",
        help="FastAPI server URL for pushing results (default: http://192.168.1.78:8000)"
    )
    parser.add_argument(
        "--exposure",
        type=int,
        default=None,
        help="Starting exposure in microseconds (default: from config)"
    )
    parser.add_argument(
        "--no-tune",
        action="store_true",
        help="Disable automatic exposure tuning"
    )
    parser.add_argument(
        "--no-upload",
        action="store_true",
        help="Skip pushing results to server -- save locally only"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=_LOCAL_CAL_DIR,
        help=f"Local output directory (default: {_LOCAL_CAL_DIR})"
    )
    args = parser.parse_args()

    server_url = None if args.no_upload else args.server

    esp32 = ESP32Controller(
        port=config.acquisition.serial_port,
        baud=config.acquisition.baud_rate
    )
    if not esp32.connect():
        print("ERROR: ESP32 connection failed -- check USB cable and serial port")
        sys.exit(1)

    camera = CameraInterface(
        width=config.acquisition.camera_width,
        height=config.acquisition.camera_height
    )

    suite = CalibrationSuite(
        esp32=esp32,
        camera=camera,
        server_url=server_url,
        work_dir=Path(args.output)
    )

    try:
        if args.mode == "all":
            result = suite.run_all(
                exposure_us=args.exposure,
                tune_exposure=not args.no_tune
            )
            sys.exit(0 if result.passed else 1)

        elif args.mode == "exposure":
            r = suite.check_exposure(exposure_us=args.exposure)
            print(f"\nExposure [{r.status.value}]: {r.message}")
            if r.recommended_exposure_us:
                print(f"  Recommended: {r.recommended_exposure_us} us")

        elif args.mode == "alignment":
            r = suite.check_alignment(exposure_us=args.exposure)
            print(f"\nAlignment [{r.status.value}]: {r.message}")
            print(f"  Centroid : ({r.centroid_x_px}, {r.centroid_y_px}) px")
            print(f"  Centre   : ({r.sensor_center_x}, {r.sensor_center_y}) px")
            print(f"  Offset   : ({r.offset_x_px:+.1f}, {r.offset_y_px:+.1f}) px")
            print(
                f"             ({r.offset_x_percent:.1f}%, {r.offset_y_percent:.1f}%)"
            )
            print(f"  Image    : {suite.work_dir / 'alignment_annotated.png'}")

        elif args.mode == "focus":
            r = suite.check_focus(exposure_us=args.exposure)
            print(f"\nFocus [{r.status.value}]: {r.message}")
            print(f"  Laplacian : {r.laplacian_score:.5f}")
            print(f"  Brenner   : {r.brenner_score:.2f}")
            print(f"  Hint      : {r.focus_direction_hint}")

    finally:
        esp32.disconnect()


if __name__ == "__main__":
    main()