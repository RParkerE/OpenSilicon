"""
FPM Image Loading Utility  —  imgload.py

Central image loading for the FPM pipeline.  All modules that previously
called cv2.imread(..., cv2.IMREAD_GRAYSCALE) should call load_uv_image()
instead.

Why this matters
────────────────
The camera (IMX219) captures color (RGB) PNG files.  With a UV LED array
(~395 nm) and a plastic slide, the raw images contain two superimposed
signals in different channels:

    Blue channel  — direct UV transmission + diffraction
                    This is the coherent signal FPM needs.
                    Dark = sample absorbs UV = amplitude constraint.
                    Bright = UV transmitted = background.

    Green + Red   — UV-induced fluorescence from the plastic slide
                    (~450–650 nm broadband emission).
                    Incoherent — cannot be used for phase retrieval.
                    Spatially decorrelated from B: corr(B,G) ≈ −0.07.

The standard cv2.IMREAD_GRAYSCALE conversion weights channels as:
    0.114·B + 0.587·G + 0.299·R
which gives 88.6% weight to the incoherent fluorescence channels and
only 11.4% to the actual UV diffraction signal.  This corrupts every
amplitude constraint in the reconstruction.

The fix is to extract only the blue channel.  The blue channel is
partially saturated (~70% of pixels at 255) because the background UV
is bright, but the unsaturated pixels (the sample absorption regions)
carry Laplacian variance of ~180 — 7× higher contrast than the G channel.
The FPM amplitude constraint is defined on the sample region, so the
saturation of the background is irrelevant.

Auto-detection
──────────────
load_uv_image() automatically detects whether the file is:
    • Color (3-channel) → extracts blue channel (BGR index 0)
    • Grayscale (1-channel) → returns as-is (no-op for old single-channel captures)
    • Missing / unreadable → returns None (same contract as cv2.imread)

No changes to calling code are needed beyond swapping the import.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional, Union

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Blue channel index in OpenCV's BGR ordering
_BLUE = 0


def load_uv_image(
    path: Union[str, Path],
    dtype: type = np.float32,
) -> Optional[np.ndarray]:
    """
    Load a UV FPM capture and return the blue channel as a 2-D array.

    This is a drop-in replacement for:
        cv2.imread(path, cv2.IMREAD_GRAYSCALE).astype(np.float32)

    For color (RGB) images:
        Returns the blue channel only.
        Blue channel = UV coherent diffraction signal.

    For grayscale images:
        Returns the single channel unchanged (backward-compatible).

    Args:
        path:  Path to the PNG file.
        dtype: Output dtype (default float32, matching pipeline convention).

    Returns:
        2-D ndarray of shape (H, W) or None if the file cannot be read.
    """
    path = str(path)
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)

    if img is None:
        logger.error(f"cv2.imread returned None for {path}")
        return None

    # Already a 2-D array (single-channel grayscale or pre-processed image)
    if img.ndim == 2:
        return img.astype(dtype)

    # 3-channel color image — extract blue channel (index 0 in BGR)
    if img.ndim == 3 and img.shape[2] >= 3:
        blue = img[:, :, _BLUE]
        return blue.astype(dtype)

    # Unexpected shape — fall back to grayscale conversion with a warning
    logger.warning(
        f"Unexpected image shape {img.shape} in {path}; "
        f"falling back to grayscale conversion"
    )
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return gray.astype(dtype)


def load_uv_image_norm(
    path: Union[str, Path],
) -> Optional[np.ndarray]:
    """
    Load and return the blue channel normalised to [0, 1].

    Convenience wrapper for reconstruction.py which works in normalised space.
    """
    img = load_uv_image(path, dtype=np.float32)
    if img is None:
        return None
    max_val = img.max()
    if max_val > 0:
        img /= max_val
    return img


def channel_report(path: Union[str, Path]) -> str:
    """
    Return a one-line diagnostic string for a captured image:
        'B:252.1(sat69%)  G:53.2  R:210.1  → using B channel'

    Used by calibration.py and diagnose.py for logging.
    """
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        return "unreadable"
    if img.ndim == 2:
        return f"grayscale mean={img.mean():.1f}"
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    sat_pct = float((b >= 254).sum()) / b.size * 100
    return (
        f"B:{b.mean():.1f}(sat{sat_pct:.0f}%)"
        f"  G:{g.mean():.1f}"
        f"  R:{r.mean():.1f}"
        f"  → using B channel"
    )