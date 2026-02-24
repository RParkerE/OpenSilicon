"""
FPM Utility Functions

Mathematical functions for k-space coordinate calculation and
optical transfer function generation.
"""

import numpy as np
from typing import Tuple


def print_kshift_table(led_pitch_m, led_height_m, wavelength_m, pixel_size_m, img_shape):
    kx, ky = get_k_coordinates(led_pitch_m, led_height_m, wavelength_m, 0.25, pixel_size_m, img_shape)
    print(f"{'LED':>4} {'kx (px)':>10} {'ky (px)':>10}")
    for i, (x, y) in enumerate(zip(kx, ky)):
        bf = "*" if i == 12 else ""
        print(f"{i+1:>4} {x:>10.2f} {y:>10.2f} {bf}")
    print(f"\nMax shift: {np.max(np.abs(kx)):.1f} px — must be < {img_shape[1]//2} px")
    
def get_k_coordinates(
    led_pitch_m: float,
    led_height_m: float,
    wavelength_m: float,
    na_obj: float,
    pixel_size_m: float,
    img_shape: Tuple[int, int]
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate k-space frequency shifts for 5×5 LED array.

    The illumination angle geometry is identical for both reflection and
    transmission modes — in both cases led_height_m is the physical distance
    from the LED plane to the sample plane, and the angle is set by the
    LED's lateral offset from the optical axis.

    The operating mode (reflection vs transmission) only affects how the
    reconstructed phase is interpreted afterwards (surface height vs optical
    thickness). It has no effect on this calculation.

    Args:
        led_pitch_m: LED centre-to-centre spacing in metres.
        led_height_m: Distance from LED array plane to sample plane in metres.
        wavelength_m: Illumination wavelength in metres.
        na_obj: Objective numerical aperture (used by caller; not used here).
        pixel_size_m: Effective pixel size at sample plane in metres.
        img_shape: Image dimensions (height, width) in pixels.

    Returns:
        Tuple of (kx_shifts_pixels, ky_shifts_pixels), each a flat array of
        length 25 giving the Fourier-pixel shift for each LED in row-major
        order (LED 1 = top-left, LED 13 = centre, LED 25 = bottom-right).
    """
    k0 = 2 * np.pi / wavelength_m

    # 5×5 grid positions: indices -2, -1, 0, +1, +2
    indices = np.arange(-2, 3)
    x_idx, y_idx = np.meshgrid(indices, indices)
    x_positions_m = x_idx * led_pitch_m
    y_positions_m = y_idx * led_pitch_m

    # Free-space geometric illumination angle.
    # sin(θ) = lateral_offset / sqrt(lateral² + height²)
    # This is exact (no small-angle approximation) and valid for the
    # corner LED angle of ~24° at the default geometry.
    r = np.sqrt(x_positions_m**2 + y_positions_m**2 + led_height_m**2)
    sin_theta_x = x_positions_m / r
    sin_theta_y = y_positions_m / r

    kx = k0 * sin_theta_x
    ky = k0 * sin_theta_y

    # Convert wave-vector (rad/m) to Fourier-pixel shift units.
    # delta_k = 2π / (N_pixels * pixel_size) — one Fourier bin per FOV.
    img_height, img_width = img_shape
    fov_x_m = img_width * pixel_size_m
    fov_y_m = img_height * pixel_size_m
    delta_kx = 2 * np.pi / fov_x_m
    delta_ky = 2 * np.pi / fov_y_m

    kx_shifts_pixels = kx / delta_kx
    ky_shifts_pixels = ky / delta_ky

    return kx_shifts_pixels.flatten(), ky_shifts_pixels.flatten()

def generate_pupil(
    img_shape: Tuple[int, int],
    na_obj: float,
    wavelength_m: float,
    pixel_size_m: float
) -> np.ndarray:
    """
    Generate objective lens pupil function (coherent transfer function).
    
    Creates a binary circular aperture in Fourier space representing the
    frequency passband of the objective lens.
    
    Args:
        img_shape: Image dimensions (height, width) in pixels
        na_obj: Objective numerical aperture
        wavelength_m: Illumination wavelength in meters
        pixel_size_m: Effective pixel size at sample plane in meters
        
    Returns:
        Complex64 array of shape img_shape, with values:
            - 1.0 inside the NA cutoff (transmitted frequencies)
            - 0.0 outside the NA cutoff (blocked frequencies)
            
    Notes:
        Cutoff spatial frequency: f_max = NA / λ
        
        This is the "circ" function in Fourier optics, representing
        the autocorrelation of the pupil function (Born & Wolf).
    """
    img_height, img_width = img_shape
    
    # Fourier frequency coordinates
    freq_x_m = np.fft.fftfreq(img_width, d=pixel_size_m)
    freq_y_m = np.fft.fftfreq(img_height, d=pixel_size_m)
    
    freq_xx, freq_yy = np.meshgrid(freq_x_m, freq_y_m)
    
    # Radial spatial frequency
    freq_radial = np.sqrt(freq_xx**2 + freq_yy**2)
    
    # Maximum spatial frequency passed by objective
    freq_max = na_obj / wavelength_m
    
    # Binary circular aperture
    pupil = (freq_radial <= freq_max).astype(np.complex64)
    
    return pupil


def calculate_theoretical_resolution(
    wavelength_m: float,
    na_effective: float
) -> float:
    """
    Calculate Abbe diffraction limit resolution.
    
    Args:
        wavelength_m: Illumination wavelength in meters
        na_effective: Effective numerical aperture (objective or synthetic)
        
    Returns:
        Theoretical resolution (minimum resolvable feature size) in meters
        
    Notes:
        Abbe formula: d = λ / (2 * NA)
        
        For FPM, NA_effective > NA_objective due to synthetic aperture.
    """
    return wavelength_m / (2 * na_effective)


def estimate_synthetic_na(
    na_obj: float,
    led_pitch_m: float,
    led_height_m: float,
    wavelength_m: float
) -> float:
    """
    Estimate synthetic numerical aperture from LED array geometry.
    
    Args:
        na_obj: Objective lens numerical aperture
        led_pitch_m: LED grid spacing in meters
        led_height_m: Distance from LED to sample in meters
        wavelength_m: Illumination wavelength in meters
        
    Returns:
        Estimated synthetic NA after FPM reconstruction
        
    Notes:
        Synthetic NA is limited by:
        1. Maximum illumination angle from corner LEDs
        2. Objective NA (sets initial passband)
        3. Overlap between shifted spectra
    """
    # Corner LED position (2 grid steps from center in both x and y)
    corner_distance_m = np.sqrt(2) * (2 * led_pitch_m)
    
    # Maximum illumination angle
    theta_max = np.arctan(corner_distance_m / led_height_m)
    
    # Synthetic NA (vectorial addition)
    na_illumination = np.sin(theta_max)
    na_synthetic = na_obj + na_illumination
    
    # Practical limit: Ensure overlap between adjacent spectra
    k_shift_max = (2 * np.pi / wavelength_m) * na_illumination
    k_obj_radius = (2 * np.pi / wavelength_m) * na_obj
    
    # If k_shift > 2 * k_obj_radius, spectra don't overlap (failure mode)
    overlap_ratio = k_obj_radius / k_shift_max  # < 0.5 means gap
    if overlap_ratio < 0.5:
        import warnings
        warnings.warn(
            f"Spectral gap detected: k_shift ({k_shift_max:.2e}) > 2*k_pupil ({2*k_obj_radius:.2e}). "
            f"FPM reconstruction will fail. Increase LED height or reduce pitch.",
            RuntimeWarning
        )
    
    return min(na_synthetic, 0.95)  # Physical limit: NA < 1 for air