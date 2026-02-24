"""
FPM System Configuration Module

Centralized configuration management using Pydantic for validation.
All physical constants and operational parameters defined here.

OPERATING MODES
---------------
transmission:
    LED array is positioned BELOW the sample. Light passes UP through the
    specimen and into the objective above. Requires a transparent or
    semi-transparent sample (glass slide, quartz, thin photoresist).
    Phase encodes optical path length through the sample:
        height_nm = (phase_rad * wavelength_nm) / (2 * pi * (n - 1))
    where n is the sample refractive index (default 1.5 for glass/resist).

reflection:
    LED array is positioned ABOVE the sample via a 50/50 beam splitter.
    Light reflects off the specimen surface and returns through the
    objective. Suitable for opaque samples (silicon wafers, metal films).
    Phase encodes surface topography via double-pass path length:
        height_nm = (phase_rad * wavelength_nm) / (4 * pi)

Both modes run full FPM reconstruction to achieve synthetic-aperture
resolution beyond the conventional Abbe diffraction limit.

IMPORTANT: Current plastic calibration target exhibits UV-induced fluorescence
at exposures >8-10 seconds. Configuration optimized to stay below threshold.
For longer exposures, use UV-transparent quartz target instead.
"""

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict
from pathlib import Path
from typing import Literal

# Valid operating modes. Referenced by client, server, and visualizer.
OperatingMode = Literal["transmission", "reflection"]


class OpticalConfig(BaseSettings):
    """
    Optical system physical parameters.
    
    These values must match the actual hardware configuration.
    Changing these requires recalibration.
    """
    
    wavelength_nm: float = Field(
        default=395.0,
        description="LED center wavelength in nanometers (UV)",
        ge=380.0,
        le=420.0
    )
    
    na_objective: float = Field(
        default=0.25,
        description="Numerical aperture of objective lens",
        gt=0.0,
        lt=1.0
    )
    
    pixel_size_um: float = Field(
        default=1.12,
        description="Camera pixel pitch in micrometers (IMX219 sensor native)",
        gt=0.0
    )
    
    magnification: float = Field(
        default=10.0,
        description="Objective magnification factor",
        gt=0.0
    )
    
    led_pitch_mm: float = Field(
        default=8.0,
        description="LED array grid spacing in millimeters (5x5 grid)",
        gt=0.0
    )
    
    led_height_mm: float = Field(
        default=50.0,
        description="Distance from LED array to sample plane in millimeters",
        gt=0.0
    )

    sample_refractive_index: float = Field(
        default=1.5,
        description=(
            "Sample refractive index — used only in transmission mode "
            "to convert phase to physical thickness. "
            "Typical values: glass/quartz=1.46, photoresist=1.52, water=1.33."
        ),
        gt=1.0,
        lt=3.0
    )
    
    @property
    def wavelength_m(self) -> float:
        """Wavelength in meters."""
        return self.wavelength_nm * 1e-9
    
    @property
    def pixel_size_m(self) -> float:
        """Effective pixel size at sample plane in meters."""
        return (self.pixel_size_um * 1e-6) / self.magnification
    
    @property
    def led_pitch_m(self) -> float:
        """LED pitch in meters."""
        return self.led_pitch_mm * 1e-3
    
    @property
    def led_height_m(self) -> float:
        """LED height in meters."""
        return self.led_height_mm * 1e-3
    
    @property
    def theoretical_resolution_nm(self) -> float:
        """Theoretical Abbe resolution limit in nanometers."""
        return (self.wavelength_nm / (2 * self.na_objective))
    
    @property
    def synthetic_na_estimate(self) -> float:
        """Estimated synthetic NA after FPM reconstruction."""
        import numpy as np
        # Corner LED angle
        corner_dist = np.sqrt(2) * (2 * self.led_pitch_mm / 1000)
        theta_max = np.arctan(corner_dist / (self.led_height_mm / 1000))
        na_illumination = np.sin(theta_max)
        return min(self.na_objective + na_illumination, 0.95)
    
    @property
    def synthetic_resolution_nm(self) -> float:
        """Theoretical resolution after FPM reconstruction in nanometers."""
        return (self.wavelength_nm / (2 * self.synthetic_na_estimate))


class ReconstructionConfig(BaseSettings):
    """
    Fourier ptychography reconstruction algorithm parameters.
    """
    
    upsample_factor: int = Field(
        default=4,
        description="Spectral resolution enhancement factor (2-8x)",
        ge=2,
        le=8
    )
    
    max_iterations: int = Field(
        default=15,
        description="Maximum Gerchberg-Saxton iterations (increased for noisy data)",
        ge=1,
        le=100
    )
    
    convergence_threshold: float = Field(
        default=1e-4,
        description="Relative change threshold for early stopping",
        gt=0.0
    )
    
    regularization_weight: float = Field(
        default=0.01,
        description="Tikhonov regularization parameter (reduces noise amplification)",
        ge=0.0,
        le=1.0
    )


class AcquisitionConfig(BaseSettings):
    """
    Image acquisition hardware parameters.
    
    FLUORESCENCE WARNING:
    Current plastic calibration target exhibits UV-induced fluorescence
    at exposures >8-10 seconds. These settings stay below threshold.
    
    For better signal quality:
    1. Use UV-transparent quartz target (allows 60-120s exposures)
    2. Switch to blue LEDs 470nm (no fluorescence, 10x faster)
    3. Use maximum gain to compensate for short exposures
    """
    
    serial_port: str = Field(
        default="/dev/ttyUSB0",
        description="ESP32 serial port device path"
    )
    
    baud_rate: int = Field(
        default=115200,
        description="Serial communication baud rate",
        gt=0
    )
    
    camera_width: int = Field(
        default=2048,
        description="Camera capture width in pixels",
        ge=512,
        le=4096
    )
    
    camera_height: int = Field(
        default=2048,
        description="Camera capture height in pixels",
        ge=512,
        le=4096
    )
    
    # EXPOSURE TIMES: Optimized for plastic target (below fluorescence threshold)
    bf_exposure_us: int = Field(
        default=9600000,
        description="Bright-field exposure in microseconds (7s - below fluorescence threshold)",
        gt=0
    )
    
    df_exposure_us: int = Field(
        default=9600000,
        description="Dark-field exposure in microseconds (7s - same as BF to avoid fluorescence)",
        gt=0
    )
    
    analog_gain: float = Field(
        default=16.0,
        description="Camera analog gain (16.0 = maximum, needed for short exposures)",
        ge=1.0,
        le=16.0
    )
    
    capture_timeout_s: int = Field(
        default=60,
        description="Maximum time to wait for single image capture (seconds)",
        gt=0
    )
    
    # NOTE: Removed exposure validator to allow longer exposures with quartz targets
    # If using quartz target, you can safely use:
    #   bf_exposure_us = 60000000  (60 seconds)
    #   df_exposure_us = 120000000 (120 seconds)


class ServerConfig(BaseSettings):
    """
    FastAPI server and storage configuration.
    """
    
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8")
    
    data_root: Path = Field(
        default=Path("./data"),
        description="Root directory for session data storage"
    )
    
    host: str = Field(
        default="0.0.0.0",
        description="Server bind address (0.0.0.0 = all interfaces)"
    )
    
    port: int = Field(
        default=8000,
        description="Server port",
        gt=0,
        lt=65536
    )
    
    log_level: Literal["DEBUG", "INFO", "WARNING", "ERROR"] = Field(
        default="INFO",
        description="Logging verbosity level"
    )
    
    max_upload_size_mb: int = Field(
        default=500,
        description="Maximum total upload size per session in megabytes",
        gt=0
    )
    
    worker_threads: int = Field(
        default=2,
        description="Number of background reconstruction worker threads",
        ge=1,
        le=8
    )
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.data_root.mkdir(parents=True, exist_ok=True)


class SystemConfig(BaseSettings):
    """
    Top-level system configuration aggregating all subsystems.

    Environment variables can override defaults using FPM_ prefix:
    - FPM_MODE=transmission
    - FPM_ACQUISITION__BF_EXPOSURE_US=60000000
    - FPM_ACQUISITION__ANALOG_GAIN=8.0
    - FPM_SERVER__DATA_ROOT=/mnt/data
    - FPM_OPTICAL__SAMPLE_REFRACTIVE_INDEX=1.46

    OPERATING MODES:

    transmission  LED below sample. Phase → optical path length through sample.
                  Height = phase * λ / (2π(n-1)).  Use for glass, resist, quartz.

    reflection    LED above sample via beam splitter. Phase → surface height.
                  Height = phase * λ / (4π).  Use for silicon, metal, opaque films.

    EXPOSURE PROFILES:

    Profile 1: Plastic Target (CURRENT)
    - bf_exposure_us = 7000000 (7s)
    - df_exposure_us = 7000000 (7s)
    - analog_gain = 16.0
    - Signal: ~20-40 mean (8-16% range) - marginal
    - Pros: Works with existing target
    - Cons: Weak signal, high noise

    Profile 2: Quartz Target (RECOMMENDED)
    - bf_exposure_us = 60000000 (60s)
    - df_exposure_us = 120000000 (120s)
    - analog_gain = 8.0
    - Signal: ~80-150 mean (30-60% range) - good
    - Pros: Strong signal, low noise
    - Cons: Requires quartz slide ($30-80)

    Profile 3: Blue LEDs (BEST LONG-TERM)
    - wavelength_nm = 470.0
    - bf_exposure_us = 1000000 (1s)
    - df_exposure_us = 3000000 (3s)
    - analog_gain = 4.0
    - Signal: ~100-200 mean (40-80% range) - excellent
    - Pros: Fast, no fluorescence, visible
    - Cons: Requires LED replacement ($10-50)
    """

    mode: OperatingMode = Field(
        default="reflection",
        description=(
            "Operating geometry. 'reflection' = LED above sample via beam "
            "splitter (opaque samples). 'transmission' = LED below sample "
            "(transparent samples). Both modes run full FPM reconstruction."
        )
    )

    optical: OpticalConfig = Field(default_factory=OpticalConfig)
    reconstruction: ReconstructionConfig = Field(default_factory=ReconstructionConfig)
    acquisition: AcquisitionConfig = Field(default_factory=AcquisitionConfig)
    server: ServerConfig = Field(default_factory=ServerConfig)

    model_config = SettingsConfigDict(env_file=".env", env_prefix="FPM_")

    def print_summary(self):
        """Print configuration summary including active mode."""
        mode_detail = {
            "reflection": (
                "LED above sample via beam splitter | "
                "Phase → surface height = φλ/(4π)"
            ),
            "transmission": (
                f"LED below sample | "
                f"Phase → thickness = φλ/(2π(n-1)), n={self.optical.sample_refractive_index}"
            ),
        }[self.mode]

        print("=" * 70)
        print("FPM SYSTEM CONFIGURATION SUMMARY")
        print("=" * 70)
        print(f"Operating mode:       {self.mode.upper()}")
        print(f"  {mode_detail}")
        print(f"\nWavelength:           {self.optical.wavelength_nm} nm")
        print(f"Objective NA:         {self.optical.na_objective}")
        print(f"Synthetic NA (est):   {self.optical.synthetic_na_estimate:.3f}")
        print(f"Theoretical res:      {self.optical.theoretical_resolution_nm:.1f} nm")
        print(f"Synthetic res (est):  {self.optical.synthetic_resolution_nm:.1f} nm")
        if self.mode == "transmission":
            print(
                f"Sample refractive idx: {self.optical.sample_refractive_index} "
                f"(set FPM_OPTICAL__SAMPLE_REFRACTIVE_INDEX to override)"
            )
        print(f"\nBright-field exp:     {self.acquisition.bf_exposure_us / 1e6:.1f}s")
        print(f"Dark-field exp:       {self.acquisition.df_exposure_us / 1e6:.1f}s")
        print(f"Analog gain:          {self.acquisition.analog_gain}x")
        print(f"\nTotal acq time (est): {self._estimate_acquisition_time():.1f} minutes")
        print(f"Data root:            {self.server.data_root}")
        print("=" * 70)
    
    def _estimate_acquisition_time(self) -> float:
        """Estimate total acquisition time in minutes."""
        # 9 bright-field LEDs, 16 dark-field LEDs
        bf_time_s = 9 * (self.acquisition.bf_exposure_us / 1e6)
        df_time_s = 16 * (self.acquisition.df_exposure_us / 1e6)
        overhead_s = 25 * 2  # 2 seconds overhead per LED
        total_s = bf_time_s + df_time_s + overhead_s
        return total_s / 60


# Global singleton instance
config = SystemConfig()


# Development/debugging helper
if __name__ == "__main__":
    config.print_summary()