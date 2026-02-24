"""
FPM Client - Raspberry Pi Acquisition Controller

Orchestrates image capture synchronized with ESP32 LED controller.
Implements hardware abstraction layer for camera and serial communication.

Usage:
    python client.py --mode reflection --session fpm_test_001 --server http://192.168.1.100:8000
    python client.py --mode transmission --session fpm_test_002 --server http://192.168.1.100:8000

Operating modes:
    reflection    LED array above sample via beam splitter.
                  Suitable for opaque/reflective samples (silicon, metal films).
                  Phase output maps to surface height via double-pass formula.

    transmission  LED array below sample, shining up through specimen.
                  Suitable for transparent samples (glass, quartz, photoresist).
                  Phase output maps to optical path length through sample.

Both modes run full FPM reconstruction to achieve sub-micron resolution.
"""

import sys
import time
import logging
import shutil
import argparse
from pathlib import Path
from typing import Optional, Set, Tuple
from dataclasses import dataclass

import serial
import subprocess
import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

from config import config, OperatingMode

# Configure module logger
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger("FPMClient")


@dataclass
class CaptureMetadata:
    """
    Metadata for single image capture.

    Attributes:
        led_index: LED position in 5x5 array (1-25)
        exposure_us: Exposure time in microseconds
        analog_gain: Camera analog gain multiplier
        timestamp_unix: Capture timestamp (Unix epoch)
        filename: Output image filename
        mode: Operating geometry ('reflection' or 'transmission')
    """
    led_index: int
    exposure_us: int
    analog_gain: float
    timestamp_unix: float
    filename: str
    mode: str


class ESP32Controller:
    """
    Serial interface to ESP32 LED matrix controller.
    
    Sends commands in protocol format: "L{led_num} E{pulse_ms}\n"
    
    Args:
        port: Serial device path (e.g., /dev/ttyUSB0)
        baud: Baud rate (must match ESP32 firmware)
        timeout: Read timeout in seconds
    """
    
    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        
    def connect(self) -> bool:
        """
        Establish serial connection to ESP32.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            time.sleep(2.0)  # Wait for ESP32 bootloader/reset
            
            # Flush any stale data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            logger.info(f"ESP32 connected on {self.port} @ {self.baud} baud")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Serial connection failed: {e}")
            return False
            
    def send_trigger(self, led_index: int, pulse_duration_ms: int) -> bool:
        """
        Send LED activation command to ESP32.
        
        Args:
            led_index: LED position (1-25)
            pulse_duration_ms: Duration LED stays active (milliseconds)
            
        Returns:
            True if command sent successfully
            
        Raises:
            RuntimeError: If serial port not connected
        """
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("ESP32 not connected. Call connect() first.")
            
        command = f"L{led_index} E{pulse_duration_ms}\n"
        
        try:
            self.ser.write(command.encode('ascii'))
            self.ser.flush()
            logger.debug(f"ESP32 command sent: {command.strip()}")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Serial write failed: {e}")
            return False
            
    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("ESP32 disconnected")


class CameraInterface:
    """
    Abstract camera control interface for rpicam/libcamera.
    
    Automatically detects available camera command (rpicam-still or libcamera-still).
    
    Args:
        width: Image width in pixels
        height: Image height in pixels
        raw_mode: Enable RAW+JPEG capture if True
    """
    
    def __init__(self, width: int = 2048, height: int = 2048, raw_mode: bool = True):
        self.width = width
        self.height = height
        self.raw_mode = raw_mode
        
        # Detect camera command
        self.cmd = shutil.which("rpicam-still") or shutil.which("libcamera-still")
        
        if not self.cmd:
            raise RuntimeError(
                "Camera software not found. Install libcamera-apps:\n"
                "  sudo apt install libcamera-apps"
            )
            
        logger.info(f"Camera command: {self.cmd}")
        
    def capture(
        self,
        output_path: Path,
        exposure_us: int,
        gain: float = 1.0,
        timeout_s: int = 30
    ) -> bool:
        """
        Capture single image with specified parameters.
        
        Args:
            output_path: Output file path (PNG format)
            exposure_us: Shutter speed in microseconds
            gain: Analog gain multiplier
            timeout_s: Command timeout in seconds
            
        Returns:
            True if capture successful, False otherwise
        """
        cmd_args = [
            self.cmd,
            "-n",  # No preview window
            "-o", str(output_path),
            "--shutter", str(exposure_us),
            "--gain", str(gain),
            "--immediate",  # Capture without delay
            "--width", str(self.width),
            "--height", str(self.height),
        ]
        
        if self.raw_mode:
            cmd_args.append("--raw")
            
        try:
            result = subprocess.run(
                cmd_args,
                check=True,
                timeout=timeout_s,
                capture_output=True,
                text=True
            )
            
            # Verify file was created
            if not output_path.exists():
                logger.error(f"Capture failed: {output_path} not created")
                return False
                
            file_size_mb = output_path.stat().st_size / (1024 * 1024)
            logger.debug(f"Captured {output_path.name} ({file_size_mb:.2f} MB)")
            return True
            
        except subprocess.TimeoutExpired:
            logger.error(f"Camera capture timeout after {timeout_s}s")
            return False
            
        except subprocess.CalledProcessError as e:
            logger.error(f"Camera command failed: {e.stderr}")
            return False


class FPMConductor:
    """
    High-level orchestrator for FPM acquisition workflow.

    Coordinates ESP32 LED control, camera capture, and server upload.

    Args:
        esp32_port: Serial port for ESP32.
        baud: Serial baud rate.
        server_url: Backend server URL (e.g., http://192.168.1.100:8000).
        mode: Operating geometry — 'reflection' (LED above sample via beam
              splitter) or 'transmission' (LED below sample). Both modes
              run full FPM reconstruction.
    """

    # Center 3x3 LEDs for bright-field imaging (lower exposure).
    # Identical for both modes: on-axis illumination is always the inner ring.
    BF_LED_INDICES: Set[int] = {7, 8, 9, 12, 13, 14, 17, 18, 19}

    # Overhead time for LED stabilization + camera setup
    OVERHEAD_MS: int = 600

    def __init__(
        self,
        esp32_port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        server_url: str = "http://localhost:8000",
        mode: OperatingMode = "reflection"
    ):
        self.mode: OperatingMode = mode
        self.esp32 = ESP32Controller(port=esp32_port, baud=baud)
        self.camera = CameraInterface(
            width=config.acquisition.camera_width,
            height=config.acquisition.camera_height
        )
        self.server_url = server_url.rstrip('/')

        # Configure requests session with retry logic
        self.session = requests.Session()
        retry_strategy = Retry(
            total=3,
            backoff_factor=1,
            status_forcelist=[500, 502, 503, 504],
            allowed_methods=["POST"]
        )
        adapter = HTTPAdapter(max_retries=retry_strategy)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)
        
    def initialize(self) -> bool:
        """
        Initialize hardware connections.
        
        Returns:
            True if all hardware ready, False otherwise
        """
        logger.info("=== FPM Conductor Initialization ===")
        
        if not self.esp32.connect():
            logger.error("ESP32 connection failed")
            return False
            
        logger.info("Hardware initialization complete")
        return True
        
    def capture_sequence(
        self,
        session_id: str,
        output_dir: Path,
        bf_exposure_us: Optional[int] = None,
        df_exposure_us: Optional[int] = None
    ) -> Tuple[int, int]:
        """
        Execute full 25-image acquisition sequence.
        
        Args:
            session_id: Unique session identifier
            output_dir: Directory for saving images
            bf_exposure_us: Bright-field exposure (uses config default if None)
            df_exposure_us: Dark-field exposure (uses config default if None)
            
        Returns:
            Tuple of (successful_captures, total_attempts)
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        
        bf_exp = bf_exposure_us or config.acquisition.bf_exposure_us
        df_exp = df_exposure_us or config.acquisition.df_exposure_us
        
        logger.info(f"=== Starting Acquisition: {session_id} ===")
        logger.info(f"Mode:   {self.mode.upper()}")
        logger.info(f"Output: {output_dir}")
        logger.info(f"Exposures: BF={bf_exp}us, DF={df_exp}us")
        
        metadata_list = []
        success_count = 0
        
        for led_idx in range(1, 26):
            exposure_us = bf_exp if led_idx in self.BF_LED_INDICES else df_exp
            pulse_ms = self.OVERHEAD_MS + (exposure_us // 1000)
            filename = f"led_{led_idx:02d}.png"
            filepath = output_dir / filename
            
            logger.info(f"Capturing LED {led_idx}/25 (Exp: {exposure_us}us)...")
            
            # Step 1: Trigger ESP32 LED activation
            if not self.esp32.send_trigger(led_idx, pulse_ms):
                logger.warning(f"ESP32 trigger failed for LED {led_idx}, skipping")
                continue
                
            # Step 2: Small delay for LED stabilization
            time.sleep(0.05)
            
            # Step 3: Capture image
            if not self.camera.capture(
                filepath,
                exposure_us=exposure_us,
                gain=config.acquisition.analog_gain,
                timeout_s=config.acquisition.capture_timeout_s
            ):
                logger.error(f"Capture failed for LED {led_idx}")
                break  # Abort sequence on critical failure
                
            # Record metadata
            metadata = CaptureMetadata(
                led_index=led_idx,
                exposure_us=exposure_us,
                analog_gain=config.acquisition.analog_gain,
                timestamp_unix=time.time(),
                filename=filename,
                mode=self.mode
            )
            metadata_list.append(metadata)
            success_count += 1
            
            # Thermal stabilization delay
            time.sleep(0.5)
            
        logger.info(f"Acquisition complete: {success_count}/25 images captured")
        
        # Save session metadata as JSON
        self._save_metadata(output_dir, session_id, metadata_list)
        
        return success_count, 25
        
    def upload_to_server(self, session_id: str, data_dir: Path) -> bool:
        """
        Upload captured images to processing server.
        
        Args:
            session_id: Session identifier
            data_dir: Directory containing images
            
        Returns:
            True if upload and processing trigger successful
        """
        image_files = sorted(data_dir.glob("led_*.png"))
        
        if not image_files:
            logger.error(f"No images found in {data_dir}")
            return False
            
        logger.info(f"Uploading {len(image_files)} images to {self.server_url}...")
        
        try:
            # Prepare multipart form data
            files = [
                ('files', (f.name, open(f, 'rb'), 'image/png'))
                for f in image_files
            ]
            
            # Upload images
            upload_url = f"{self.server_url}/upload/{session_id}"
            response = self.session.post(upload_url, files=files, timeout=300)
            response.raise_for_status()
            
            logger.info("Upload successful")
            
            # Trigger reconstruction — pass operating mode so server stores
            # it in status.json and visualizer can read it for phase conversion.
            process_url = f"{self.server_url}/process/{session_id}"
            response = self.session.post(
                process_url,
                params={"mode": self.mode},
                timeout=10
            )
            response.raise_for_status()
            
            logger.info("Reconstruction queued on server")
            return True
            
        except requests.exceptions.RequestException as e:
            logger.error(f"Server communication failed: {e}")
            return False
            
        finally:
            # Close file handles
            for _, (_, f, _) in files:
                f.close()
                
    def _save_metadata(
        self,
        output_dir: Path,
        session_id: str,
        metadata_list: list
    ):
        """
        Save acquisition metadata as JSON manifest.
        
        Args:
            output_dir: Output directory
            session_id: Session identifier
            metadata_list: List of CaptureMetadata objects
        """
        import json
        from dataclasses import asdict
        
        manifest = {
            "session_id": session_id,
            "mode": self.mode,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "frames_captured": len(metadata_list),
            "optical_config": {
                "wavelength_nm": config.optical.wavelength_nm,
                "na_objective": config.optical.na_objective,
                "magnification": config.optical.magnification,
                "sample_refractive_index": config.optical.sample_refractive_index,
                "led_height_mm": config.optical.led_height_mm,
            },
            "captures": [asdict(m) for m in metadata_list]
        }
        
        manifest_path = output_dir / f"{session_id}_manifest.json"
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
            
        logger.info(f"Metadata saved: {manifest_path}")
        
    def shutdown(self):
        """Release hardware resources."""
        self.esp32.disconnect()
        self.session.close()
        logger.info("Conductor shutdown complete")


def main():
    """CLI entry point for FPM client."""
    parser = argparse.ArgumentParser(
        description="FPM Acquisition Client for Raspberry Pi"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["reflection", "transmission"],
        default=config.mode,
        help=(
            "Operating geometry. "
            "'reflection' = LED above sample via beam splitter (opaque samples). "
            "'transmission' = LED below sample (transparent samples). "
            f"(default from config: {config.mode})"
        )
    )
    parser.add_argument(
        "--session",
        type=str,
        default=f"fpm_{int(time.time())}",
        help="Session identifier (default: fpm_<timestamp>)"
    )
    parser.add_argument(
        "--server",
        type=str,
        default="http://0.0.0.0:8000",
        help="Server URL (default: http://0.0.0.0:8000)"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("./captures"),
        help="Local output directory (default: ./captures)"
    )
    parser.add_argument(
        "--no-upload",
        action="store_true",
        help="Skip server upload (capture only)"
    )

    args = parser.parse_args()

    logger.info(f"Operating mode: {args.mode.upper()}")

    # Initialize conductor with selected mode
    conductor = FPMConductor(
        esp32_port=config.acquisition.serial_port,
        baud=config.acquisition.baud_rate,
        server_url=args.server,
        mode=args.mode
    )
    
    try:
        if not conductor.initialize():
            logger.error("Hardware initialization failed")
            sys.exit(1)
            
        # Execute acquisition
        session_dir = args.output / args.session
        success, total = conductor.capture_sequence(
            session_id=args.session,
            output_dir=session_dir
        )
        
        if success < total:
            logger.warning(f"Incomplete acquisition: {success}/{total} images")
            
        # Upload to server
        if not args.no_upload and success > 0:
            if conductor.upload_to_server(args.session, session_dir):
                logger.info("Workflow complete")
            else:
                logger.error("Upload failed - data saved locally")
                
    except KeyboardInterrupt:
        logger.warning("Acquisition interrupted by user")
        
    except Exception as e:
        logger.exception(f"Unexpected error: {e}")
        sys.exit(1)
        
    finally:
        conductor.shutdown()


if __name__ == "__main__":
    main()