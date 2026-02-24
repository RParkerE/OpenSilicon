# Copyright (c) 2025 FPM Metrology Project
# SPDX-License-Identifier: MIT
#
# FPM Optical Alignment Analysis Module
#
# Corrected alignment analysis using bright-spot Gaussian centroid
# instead of darkest-pixel detection.

"""
FPM Alignment Analysis Module

Computes measured LED illumination centroids from captured image stack
and compares against geometrically predicted k-space positions.

Correct method: Gaussian-weighted centroid of the bright illumination
hotspot. Each LED produces a Gaussian intensity peak whose centroid
encodes the incident k-vector at the sample plane.
"""

import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from config import SystemConfig
from imgload import load_uv_image
from utils import get_k_coordinates

logger = logging.getLogger(__name__)


class AlignmentAnalyzer:
    """
    Measures optical alignment by locating LED illumination centroids.

    For each LED image, computes the Gaussian-weighted centroid of the
    bright illumination region. Compares measured positions against
    geometrically predicted positions from LED array geometry.

    Attributes:
        config: System configuration instance.
        image_center: (cx, cy) image center coordinates in pixels.
    """

    def __init__(self, config: SystemConfig):
        """
        Initialize analyzer with system configuration.

        Args:
            config: SystemConfig instance with optical parameters.
        """
        self.config = config
        camera_w = config.acquisition.camera_width
        camera_h = config.acquisition.camera_height
        self.image_center = (camera_w // 2, camera_h // 2)

        logger.info(
            "AlignmentAnalyzer initialized. "
            f"Image center: {self.image_center}"
        )

    def _find_bright_centroid(
        self, image_gray: np.ndarray
    ) -> Tuple[float, float, float]:
        """
        Locate illumination hotspot via Gaussian-weighted centroid.

        Applies a percentile-based threshold to isolate the bright
        illumination region, then computes the intensity-weighted
        center of mass. This is robust against dust and fixed-pattern
        defects because those are dark, not bright.

        Args:
            image_gray: Single-channel float32 image.

        Returns:
            Tuple of (centroid_x, centroid_y, peak_intensity).
            Centroid is sub-pixel accurate via weighted moment.

        Raises:
            ValueError: If no valid bright region is found.
        """
        # Threshold at the 95th percentile to isolate the bright spot.
        # This rejects dust (dark) and fixed-pattern noise (mid-level).
        threshold = np.percentile(image_gray, 95)
        bright_mask = image_gray > threshold

        if not np.any(bright_mask):
            raise ValueError("No bright region found above 95th percentile threshold.")

        # Intensity-weighted centroid (moment calculation)
        masked_intensity = image_gray * bright_mask.astype(np.float32)
        total_weight = masked_intensity.sum()

        if total_weight < 1e-6:
            raise ValueError("Bright region has negligible total intensity.")

        # y-axis moment
        row_indices = np.arange(image_gray.shape[0]).reshape(-1, 1)
        col_indices = np.arange(image_gray.shape[1]).reshape(1, -1)

        centroid_y = (masked_intensity * row_indices).sum() / total_weight
        centroid_x = (masked_intensity * col_indices).sum() / total_weight
        peak_intensity = image_gray[bright_mask].max()

        return float(centroid_x), float(centroid_y), float(peak_intensity)

    def _compute_predicted_positions(
        self, image_shape: Tuple[int, int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute predicted centroid positions in pixel coordinates.

        Converts the geometric k-space shifts (from get_k_coordinates)
        back to image-plane pixel offsets. The center LED (index 12)
        should produce a centroid at the image center; outer LEDs
        shift proportionally to their illumination angle.

        Args:
            image_shape: (height, width) of the image.

        Returns:
            Tuple of (predicted_x_pixels, predicted_y_pixels),
            both 1D arrays of length 25, in absolute pixel coordinates.
        """
        kx_shifts, ky_shifts = get_k_coordinates(
            self.config.optical.led_pitch_m,
            self.config.optical.led_height_m,
            self.config.optical.wavelength_m,
            self.config.optical.na_objective,
            self.config.optical.pixel_size_m,
            image_shape
        )

        cx, cy = self.image_center

        # k-space shifts are in Fourier-pixel units; in the spatial domain
        # the illumination centroid shifts by the same amount (reciprocal space
        # shift = spatial carrier frequency = centroid displacement in the
        # incoherent imaging approximation used here for alignment).
        predicted_x = cx + kx_shifts
        predicted_y = cy + ky_shifts

        return predicted_x, predicted_y

    def analyze_session(
        self, session_path: Path
    ) -> Dict:
        """
        Run full alignment analysis on a captured session.

        Loads all 25 LED images, extracts bright-spot centroids,
        computes predicted positions, and returns a structured report
        including per-LED residuals and overall alignment quality.

        Args:
            session_path: Path to session directory containing led_XX.png files.

        Returns:
            Dictionary with keys:
                - session: session directory name
                - method: 'bright_centroid'
                - predicted_positions: list of {led, pred_x, pred_y}
                - per_led_results: dict keyed by LED index (1-25)
                - statistics: dict with mean/median/max residuals
                - alignment_quality: 'GOOD' | 'MARGINAL' | 'POOR'

        Raises:
            FileNotFoundError: If session directory does not exist.
            ValueError: If fewer than 25 images are present.
        """
        if not session_path.exists():
            raise FileNotFoundError(f"Session not found: {session_path}")

        image_files = sorted(session_path.glob("led_*.png"))
        if len(image_files) < 25:
            raise ValueError(
                f"Incomplete session: {len(image_files)}/25 images found."
            )

        logger.info(
            f"Analyzing session: {session_path.name} "
            f"({len(image_files)} images)"
        )

        # Load first image to establish shape
        first_image = load_uv_image(image_files[0])
        image_shape = first_image.shape  # (height, width)

        predicted_x, predicted_y = self._compute_predicted_positions(image_shape)

        per_led_results = {}
        residuals = []

        for led_number in range(1, 26):
            filename = f"led_{led_number:02d}.png"
            image_path = session_path / filename

            if not image_path.exists():
                logger.warning(f"Missing image: {filename}")
                continue

            image_gray = load_uv_image(image_path)

            try:
                centroid_x, centroid_y, peak = self._find_bright_centroid(image_gray)
            except ValueError as error:
                logger.error(f"LED {led_number}: centroid failed — {error}")
                continue

            # LED index 0-based for array lookups
            led_idx = led_number - 1
            pred_x = predicted_x[led_idx]
            pred_y = predicted_y[led_idx]

            # Residual: measured centroid vs. geometric prediction
            residual_x = centroid_x - pred_x
            residual_y = centroid_y - pred_y
            residual_distance = float(
                np.sqrt(residual_x**2 + residual_y**2)
            )

            # Offset from image center (legacy metric, kept for comparison)
            cx, cy = self.image_center
            offset_x = centroid_x - cx
            offset_y = centroid_y - cy
            distance_from_center = float(
                np.sqrt(offset_x**2 + offset_y**2)
            )

            residuals.append(residual_distance)

            per_led_results[str(led_number)] = {
                "led_index": led_number,
                "filename": filename,
                "method": "bright_centroid",
                "centroid_x": round(centroid_x, 2),
                "centroid_y": round(centroid_y, 2),
                "predicted_x": round(pred_x, 2),
                "predicted_y": round(pred_y, 2),
                "residual_x": round(residual_x, 2),
                "residual_y": round(residual_y, 2),
                "residual_distance_pixels": round(residual_distance, 2),
                "offset_from_center_x": round(offset_x, 2),
                "offset_from_center_y": round(offset_y, 2),
                "distance_from_center_pixels": round(distance_from_center, 2),
                "peak_intensity": round(peak, 1),
                "mean_intensity": round(float(image_gray.mean()), 3),
                "std_intensity": round(float(image_gray.std()), 3)
            }

            logger.debug(
                f"LED {led_number:2d}: centroid=({centroid_x:.1f},{centroid_y:.1f}), "
                f"predicted=({pred_x:.1f},{pred_y:.1f}), "
                f"residual={residual_distance:.1f}px"
            )

        residuals_array = np.array(residuals)
        alignment_quality = self._classify_alignment(residuals_array)

        result = {
            "session": session_path.name,
            "method": "bright_centroid_weighted_moment",
            "image_shape": list(image_shape),
            "image_center": list(self.image_center),
            "statistics": {
                "mean_residual_pixels": round(float(residuals_array.mean()), 3),
                "median_residual_pixels": round(
                    float(np.median(residuals_array)), 3
                ),
                "max_residual_pixels": round(float(residuals_array.max()), 3),
                "min_residual_pixels": round(float(residuals_array.min()), 3),
                "std_residual_pixels": round(float(residuals_array.std()), 3),
                "total_images": len(per_led_results)
            },
            "alignment_quality": alignment_quality,
            "per_led_results": per_led_results
        }

        logger.info(
            f"Alignment analysis complete. "
            f"Mean residual: {residuals_array.mean():.1f}px, "
            f"Quality: {alignment_quality}"
        )

        return result

    def _classify_alignment(self, residuals: np.ndarray) -> str:
        """
        Classify overall alignment quality from residual statistics.

        Thresholds are expressed in Fourier pixels. For FPM to work
        correctly, the spectral overlap between adjacent LEDs must be
        maintained. A residual larger than the objective NA radius in
        Fourier pixels indicates the spectra may not overlap correctly.

        Args:
            residuals: Array of per-LED residual distances in pixels.

        Returns:
            Quality string: 'GOOD', 'MARGINAL', or 'POOR'.
        """
        # NA radius in Fourier pixels: sets the minimum acceptable overlap
        na_radius_pixels = (
            self.config.optical.na_objective
            / self.config.optical.wavelength_m
            * self.config.optical.pixel_size_m
            * self.config.acquisition.camera_width
        )
        mean_residual = residuals.mean()

        # GOOD: mean residual < 20% of NA radius (sub-pixel spectral placement)
        # MARGINAL: residual between 20% and 50% of NA radius
        # POOR: residual > 50% of NA radius (spectra may not overlap)
        if mean_residual < 0.20 * na_radius_pixels:
            return "GOOD"
        elif mean_residual < 0.50 * na_radius_pixels:
            return "MARGINAL"
        else:
            return "POOR"


def run_alignment_analysis(
    session_path: Path,
    output_path: Optional[Path] = None,
    config: Optional[SystemConfig] = None
) -> Dict:
    """
    Convenience function to run alignment analysis and save results.

    Args:
        session_path: Path to session directory.
        output_path: Optional path for JSON output. Defaults to
            session_path / 'alignment_results_corrected.json'.
        config: Optional SystemConfig. Defaults to global config.

    Returns:
        Analysis result dictionary.
    """
    if config is None:
        from config import config as global_config
        config = global_config

    if output_path is None:
        output_path = session_path / "alignment_results_corrected.json"

    analyzer = AlignmentAnalyzer(config)
    results = analyzer.analyze_session(session_path)

    with open(output_path, "w") as output_file:
        json.dump(results, output_file, indent=2)

    logger.info(f"Results saved to: {output_path}")
    return results


if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.DEBUG)

    if len(sys.argv) < 2:
        print("Usage: python alignment_analysis.py <session_path>")
        sys.exit(1)

    results = run_alignment_analysis(Path(sys.argv[1]))
    print(json.dumps(results["statistics"], indent=2))
    print(f"Alignment quality: {results['alignment_quality']}")