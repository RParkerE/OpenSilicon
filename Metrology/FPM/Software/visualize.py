"""
FPM Visualization Dashboard

Streamlit dashboard for browsing FPM reconstruction results.
Reads the operating mode from each session's status.json and applies
the correct phase-to-height conversion formula:

    reflection:   h = φλ / (4π)
        Light travels down to the surface and back up (double pass).
        Phase encodes surface topography in nm.

    transmission: h = φλ / (2π(n-1))
        Light passes through the sample once (single pass).
        Phase encodes optical path length; dividing by (n-1) gives
        physical thickness. Refractive index n is read from the session
        manifest, defaulting to the value in config.py.
"""

import os
import json
import logging
from pathlib import Path
from typing import Optional, Tuple

import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go

from config import config, OperatingMode

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

st.set_page_config(
    layout="wide",
    page_title="R-FPM Analysis Dashboard",
    page_icon="🔬",
    initial_sidebar_state="expanded"
)

# Mode display configuration
MODE_CONFIG = {
    "reflection": {
        "label": "Reflection",
        "icon": "🪞",
        "color": "#4A90D9",
        "sample_type": "Opaque / Reflective (silicon, metal film)",
        "height_label": "Surface Height (nm)",
        "formula": "h = φλ / (4π)  [double-pass]",
    },
    "transmission": {
        "label": "Transmission",
        "icon": "💡",
        "color": "#27AE60",
        "sample_type": "Transparent (glass, quartz, photoresist)",
        "height_label": "Optical Thickness (nm)",
        "formula": "h = φλ / (2π(n–1))  [single-pass]",
    },
}


def load_session_mode(session_path: Path) -> Tuple[OperatingMode, float]:
    """
    Read operating mode and refractive index from session status.json
    or manifest file.

    Falls back to config defaults if neither file is present (handles
    legacy sessions captured before mode tracking was added).

    Args:
        session_path: Path to session directory.

    Returns:
        Tuple of (mode, sample_refractive_index).
    """
    # Primary: status.json written by server at reconstruction time
    status_file = session_path / "status.json"
    if status_file.exists():
        with open(status_file) as f:
            status = json.load(f)
        mode = status.get("mode", config.mode)
        n = status.get("sample_refractive_index", config.optical.sample_refractive_index)
        return mode, n

    # Secondary: manifest written by client at capture time
    manifests = list(session_path.glob("*_manifest.json"))
    if manifests:
        with open(manifests[0]) as f:
            manifest = json.load(f)
        mode = manifest.get("mode", config.mode)
        n = manifest.get("optical_config", {}).get(
            "sample_refractive_index",
            config.optical.sample_refractive_index
        )
        return mode, n

    # Fallback for legacy sessions
    logger.warning(
        f"No status.json or manifest found in {session_path}. "
        f"Defaulting to mode='{config.mode}'."
    )
    return config.mode, config.optical.sample_refractive_index


def load_reconstruction_images(
    session_path: Path
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Load amplitude and phase reconstruction results.
    Handles both current and legacy naming conventions.

    Args:
        session_path: Path to session directory.

    Returns:
        Tuple of (amplitude_array, phase_array), either may be None.
    """
    amp_file = session_path / "reconstruction_amplitude.tiff"
    phase_file = session_path / "reconstruction_phase.tiff"

    # Legacy naming fallback
    if not amp_file.exists():
        amp_file = session_path / "result_amplitude.tiff"
    if not phase_file.exists():
        phase_file = session_path / "result_phase_map.tiff"

    amplitude = None
    phase = None

    if amp_file.exists():
        amplitude = cv2.imread(str(amp_file), cv2.IMREAD_UNCHANGED)
        logger.info(f"Loaded amplitude: {amp_file}")
    else:
        logger.warning(f"Amplitude file not found: {amp_file}")

    if phase_file.exists():
        phase = cv2.imread(str(phase_file), cv2.IMREAD_UNCHANGED)
        logger.info(f"Loaded phase: {phase_file}")
    else:
        logger.warning(f"Phase file not found: {phase_file}")

    return amplitude, phase


def phase_to_height(
    phase_map: np.ndarray,
    wavelength_nm: float,
    mode: OperatingMode,
    refractive_index: float = 1.5
) -> np.ndarray:
    """
    Convert 16-bit encoded phase map to physical height or thickness.

    The phase is first decoded from uint16 storage back to radians in
    the range [-π, π], then converted using the formula appropriate for
    the operating mode.

    Reflection (double-pass path length):
        φ = 4π·h / λ  →  h = φλ / (4π)

    Transmission (single-pass optical path length through sample):
        φ = 2π·(n-1)·t / λ  →  t = φλ / (2π·(n-1))
        where n is the sample refractive index and t is physical thickness.

    Args:
        phase_map: 16-bit uint phase image (0=−π, 65535=+π).
        wavelength_nm: Illumination wavelength in nanometers.
        mode: 'reflection' or 'transmission'.
        refractive_index: Sample refractive index (transmission mode only).

    Returns:
        Height or thickness array in nanometers, same shape as phase_map.
    """
    phase_rad = (phase_map.astype(np.float32) / 65535.0) * (2 * np.pi) - np.pi

    if mode == "reflection":
        # Double-pass: light travels to surface and back
        height_nm = (phase_rad * wavelength_nm) / (4 * np.pi)
    else:
        # Single-pass: light travels through sample thickness t
        # φ = 2π(n-1)t/λ  →  t = φλ / (2π(n-1))
        n_minus_1 = refractive_index - 1.0
        if n_minus_1 <= 0:
            raise ValueError(
                f"Refractive index must be > 1.0, got {refractive_index}"
            )
        height_nm = (phase_rad * wavelength_nm) / (2 * np.pi * n_minus_1)

    return height_nm


def create_3d_surface(
    height_map: np.ndarray,
    downsample_factor: int = 4,
    colorscale: str = "Viridis",
    height_label: str = "Height (nm)"
) -> go.Figure:
    """
    Create interactive 3D surface plot.

    Args:
        height_map: 2D array of height/thickness values in nm.
        downsample_factor: Spatial decimation factor for performance.
        colorscale: Plotly colorscale name.
        height_label: Z-axis label (varies by mode).

    Returns:
        Plotly Figure object.
    """
    z_data = height_map[::downsample_factor, ::downsample_factor]
    x = np.arange(z_data.shape[1])
    y = np.arange(z_data.shape[0])

    fig = go.Figure(data=[
        go.Surface(
            z=z_data,
            x=x,
            y=y,
            colorscale=colorscale,
            colorbar=dict(title=height_label, titleside="right")
        )
    ])

    fig.update_layout(
        title="3D Surface Topography",
        autosize=True,
        scene=dict(
            xaxis=dict(title="X (pixels)"),
            yaxis=dict(title="Y (pixels)"),
            zaxis=dict(title=height_label),
            aspectmode="manual",
            aspectratio=dict(x=1, y=1, z=0.3)
        ),
        margin=dict(l=0, r=0, b=0, t=40)
    )

    return fig


def main():
    st.title("🔬 R-FPM Reconstruction Viewer")
    st.markdown("*Fourier Ptychography Microscope — High-Resolution Analysis Dashboard*")

    # ── Sidebar: Session selector ──────────────────────────────────────
    st.sidebar.header("📁 Session Browser")

    data_root = Path(config.server.data_root)

    if not data_root.exists():
        st.error(f"Data directory not found: {data_root}")
        st.info(f"Expected path: {data_root.absolute()}")
        return

    sessions = sorted(
        [d.name for d in data_root.iterdir() if d.is_dir()],
        reverse=True
    )

    if not sessions:
        st.warning("No acquisition sessions found.")
        st.info(f"Looking in: {data_root.absolute()}")
        return

    selected_session = st.sidebar.selectbox("Select Session", sessions)

    if not selected_session:
        return

    session_path = data_root / selected_session

    # Read mode from session files — determines phase formula and UI labels
    mode, refractive_index = load_session_mode(session_path)
    mc = MODE_CONFIG[mode]

    # ── Mode badge ─────────────────────────────────────────────────────
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚙️ Operating Mode")
    st.sidebar.markdown(
        f"<div style='background:{mc['color']};padding:8px 12px;border-radius:6px;"
        f"color:white;font-weight:bold;font-size:1.05em;'>"
        f"{mc['icon']} {mc['label']}</div>",
        unsafe_allow_html=True
    )
    st.sidebar.markdown(f"**Sample:** {mc['sample_type']}")
    st.sidebar.markdown(f"**Formula:** `{mc['formula']}`")
    if mode == "transmission":
        st.sidebar.markdown(f"**Refractive index (n):** {refractive_index:.3f}")

    st.sidebar.markdown("---")
    st.sidebar.subheader("📊 Session Info")
    image_count = len(list(session_path.glob("led_*.png")))
    st.sidebar.text(f"Images: {image_count}/25")

    # ── Main content ───────────────────────────────────────────────────
    st.header(f"Session: `{selected_session}`")

    # Mode banner in main area
    st.markdown(
        f"<div style='background:{mc['color']}22;border-left:4px solid {mc['color']};"
        f"padding:10px 16px;border-radius:4px;margin-bottom:16px;'>"
        f"<b>{mc['icon']} {mc['label']} Mode</b> &nbsp;·&nbsp; {mc['sample_type']}<br>"
        f"<small>Phase → {mc['height_label']}: <code>{mc['formula']}</code></small>"
        f"</div>",
        unsafe_allow_html=True
    )

    with st.expander("🔍 Debug Info"):
        st.code(f"Session path: {session_path.absolute()}")
        st.code(f"Mode: {mode}  |  n = {refractive_index}")
        st.text("Files in session:")
        for f in sorted(session_path.iterdir()):
            st.text(f"  - {f.name}")

    # Load reconstruction results
    amplitude, phase = load_reconstruction_images(session_path)

    if amplitude is None or phase is None:
        st.warning("⚠️ Reconstruction not complete")

        status_file = session_path / "status.json"
        if status_file.exists():
            with open(status_file) as f:
                status_data = json.load(f)
            st.json(status_data)
        else:
            st.info("No status information available. Check server logs.")

        return

    st.success("✅ Reconstruction Complete")

    # Metrics row
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.metric("Resolution", f"{amplitude.shape[0]} × {amplitude.shape[1]}")
    with col2:
        st.metric("Bit Depth", "16-bit")
    with col3:
        st.metric("Images Used", image_count)
    with col4:
        st.metric("Mode", mc["label"])

    st.markdown("---")

    # ── Tabs ───────────────────────────────────────────────────────────
    tab1, tab2, tab3 = st.tabs([
        "📈 Amplitude",
        "🌈 Phase Map",
        f"🗻 3D {mc['height_label'].split('(')[0].strip()}"
    ])

    with tab1:
        st.subheader("High-Resolution Amplitude")
        amp_normalized = (amplitude / amplitude.max() * 255).astype(np.uint8)
        st.image(amp_normalized, use_column_width=True, clamp=True)

        c1, c2, c3 = st.columns(3)
        with c1:
            st.metric("Mean", f"{amplitude.mean():.0f}")
        with c2:
            st.metric("Std Dev", f"{amplitude.std():.0f}")
        with c3:
            st.metric("Max", f"{amplitude.max():.0f}")

    with tab2:
        st.subheader("Phase Distribution")
        phase_normalized = (phase / 65535.0 * 255).astype(np.uint8)
        phase_colored = cv2.applyColorMap(phase_normalized, cv2.COLORMAP_JET)
        st.image(phase_colored, use_container_width=True, channels="BGR")

        phase_rad = (phase.astype(np.float32) / 65535.0) * (2 * np.pi) - np.pi
        c1, c2 = st.columns(2)
        with c1:
            st.metric("Mean Phase", f"{phase_rad.mean():.3f} rad")
        with c2:
            st.metric("Phase Range", f"{phase_rad.max() - phase_rad.min():.3f} rad")

    with tab3:
        st.subheader(f"3D {mc['height_label']}")

        # Allow user to override refractive index in transmission mode
        if mode == "transmission":
            n_override = st.number_input(
                "Refractive index (n) — override for this view",
                min_value=1.01,
                max_value=3.0,
                value=float(refractive_index),
                step=0.01,
                help=(
                    "Change n to adjust the physical thickness scale. "
                    "glass/quartz ≈ 1.46, photoresist ≈ 1.52, water ≈ 1.33"
                )
            )
        else:
            n_override = refractive_index

        height_map = phase_to_height(
            phase,
            config.optical.wavelength_nm,
            mode=mode,
            refractive_index=n_override
        )

        downsample = st.slider(
            "Render quality (lower = faster)",
            min_value=2, max_value=10, value=4, step=1
        )

        with st.spinner("Rendering 3D surface..."):
            fig = create_3d_surface(
                height_map,
                downsample_factor=downsample,
                height_label=mc["height_label"]
            )
            st.plotly_chart(fig, use_container_width=True)

        c1, c2, c3 = st.columns(3)
        with c1:
            st.metric(f"Mean {mc['height_label']}", f"{height_map.mean():.2f} nm")
        with c2:
            st.metric("RMS Roughness", f"{np.sqrt(np.mean(height_map**2)):.2f} nm")
        with c3:
            st.metric("Peak-to-Valley", f"{height_map.max() - height_map.min():.2f} nm")


if __name__ == "__main__":
    main()