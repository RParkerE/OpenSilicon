import os
import cv2
import numpy as np
import logging
import json
import aiofiles
from imgload import load_uv_image
from pathlib import Path
from fastapi import FastAPI, UploadFile, File, BackgroundTasks, HTTPException, Query
from typing import List

from config import config, OperatingMode
from reconstruction import FPMReconstructor
from utils import get_k_coordinates

# Initialize logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = FastAPI(title="R-FPM Metrology Server")
reconstructor = FPMReconstructor(config)


def run_analysis(session_id: str, mode: OperatingMode):
    """
    Orchestrates the loading, processing, and saving of FPM data.

    Args:
        session_id: Session directory name.
        mode: Operating geometry — 'reflection' or 'transmission'.
              Stored in status.json so the visualizer can apply the
              correct phase-to-height conversion formula.
    """
    session_path = os.path.join(config.server.data_root, session_id)
    status_file = os.path.join(session_path, "status.json")

    # Write initial status — include mode so visualizer can read it
    # without needing a separate manifest lookup.
    status_data = {
        "status": "processing",
        "mode": mode,
        "frames_uploaded": len(list(Path(session_path).glob("led_*.png")))
    }

    with open(status_file, 'w') as f:
        json.dump(status_data, f, indent=2)

    try:
        logger.info(f"Processing session: {session_id}  mode={mode}")

        # 1. Load Image Stack
        img_files = sorted([f for f in os.listdir(session_path) if f.endswith('.png')])
        if len(img_files) < 25:
            logger.error(f"Incomplete stack for {session_id}: found {len(img_files)} images.")
            raise ValueError(f"Incomplete stack: {len(img_files)}/25 images")

        images = [
            load_uv_image(os.path.join(session_path, f))
            for f in img_files
        ]
        img_shape = images[0].shape

        # 2. Coordinate Mapping
        # k-space geometry is identical for both modes — only the physical
        # LED position relative to the sample matters, not which side it is on.
        kx, ky = get_k_coordinates(
            config.optical.led_pitch_m,
            config.optical.led_height_m,
            config.optical.wavelength_m,
            config.optical.na_objective,
            config.optical.pixel_size_m,
            img_shape
        )

        # 3. Execute Reconstruction (identical algorithm for both modes)
        amplitude, phase, metadata = reconstructor.reconstruct(images, kx, ky)

        # 4. Save results
        amp_scaled = (amplitude / np.max(amplitude) * 65535).astype(np.uint16)
        phase_scaled = ((phase + np.pi) / (2 * np.pi) * 65535).astype(np.uint16)

        cv2.imwrite(os.path.join(session_path, "reconstruction_amplitude.tiff"), amp_scaled)
        cv2.imwrite(os.path.join(session_path, "reconstruction_phase.tiff"), phase_scaled)

        logger.info(f"Reconstruction successful for {session_id}")

        # Update status: Complete — preserve mode field
        status_data.update({
            "status": "complete",
            "reconstruction_complete": True,
            "convergence_iterations": metadata.get("iterations", None),
            "final_relative_change": metadata.get("final_relative_change", None),
        })

        with open(status_file, 'w') as f:
            json.dump(status_data, f, indent=2)

    except Exception as e:
        logger.error(f"Reconstruction failed for {session_id}: {str(e)}")

        status_data.update({
            "status": "failed",
            "error_message": str(e)
        })

        with open(status_file, 'w') as f:
            json.dump(status_data, f, indent=2)


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "service": "R-FPM Metrology Server",
        "version": "1.0.0",
        "endpoints": {
            "health": "/health",
            "upload": "/upload/{session_id}",
            "process": "/process/{session_id}",
            "sessions": "/sessions",
            "status": "/sessions/{session_id}/status"
        }
    }


@app.get("/health")
async def health_check():
    """Server health check endpoint."""
    return {
        "status": "healthy",
        "version": "1.0.0",
        "data_root": config.server.data_root,
        "default_mode": config.mode,
        "wavelength_nm": config.optical.wavelength_nm,
        "na_objective": config.optical.na_objective
    }


@app.get("/sessions")
async def list_sessions():
    """List all available acquisition sessions."""
    data_root = Path(config.server.data_root)

    if not data_root.exists():
        return []

    sessions = [
        d.name for d in sorted(
            data_root.iterdir(),
            key=lambda x: x.stat().st_mtime,
            reverse=True
        )
        if d.is_dir()
    ]

    return sessions


@app.get("/sessions/{session_id}/status")
async def get_session_status(session_id: str):
    """Query reconstruction status for specific session."""
    session_path = Path(config.server.data_root) / session_id
    status_file = session_path / "status.json"

    if not session_path.exists():
        raise HTTPException(status_code=404, detail=f"Session '{session_id}' not found")

    if status_file.exists():
        with open(status_file, 'r') as f:
            status_data = json.load(f)
    else:
        status_data = {"status": "pending"}

    frames_uploaded = len(list(session_path.glob("led_*.png")))
    reconstruction_complete = (session_path / "reconstruction_amplitude.tiff").exists()

    return {
        "session_id": session_id,
        "status": status_data.get("status", "pending"),
        "mode": status_data.get("mode", config.mode),
        "frames_uploaded": frames_uploaded,
        "frames_expected": 25,
        "reconstruction_complete": reconstruction_complete,
        "error_message": status_data.get("error_message")
    }


@app.post("/upload/{session_id}")
async def upload_data(session_id: str, files: List[UploadFile] = File(...)):
    """Upload image stack from Raspberry Pi client."""
    session_path = os.path.join(config.server.data_root, session_id)
    os.makedirs(session_path, exist_ok=True)

    for file in files:
        file_path = os.path.join(session_path, file.filename)
        async with aiofiles.open(file_path, 'wb') as out_file:
            content = await file.read()
            await out_file.write(content)

    logger.info(f"Uploaded {len(files)} images to {session_id}")
    return {"status": "success", "session": session_id}


@app.post("/process/{session_id}")
async def process_session(
    session_id: str,
    background_tasks: BackgroundTasks,
    mode: OperatingMode = Query(
        default=config.mode,
        description=(
            "Operating geometry for this session. "
            "'reflection' = LED above sample via beam splitter. "
            "'transmission' = LED below sample. "
            "Stored in status.json; used by visualizer for phase-to-height conversion."
        )
    )
):
    """
    Trigger background reconstruction for uploaded session.

    The mode parameter is stored in status.json alongside reconstruction
    results and is read by the Streamlit dashboard to select the correct
    phase-to-height conversion formula.
    """
    background_tasks.add_task(run_analysis, session_id, mode)
    return {
        "status": "accepted",
        "mode": mode,
        "message": f"Reconstruction queued in background ({mode} mode)"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)