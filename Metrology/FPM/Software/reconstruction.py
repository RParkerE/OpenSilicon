"""
FPM Reconstruction Module

Implements iterative Fourier ptychographic phase retrieval algorithm
for super-resolution image synthesis.

Algorithm: Gerchberg-Saxton with Fourier constraint updates
Reference: Zheng et al., Nature Photonics 7, 739–745 (2013)
"""

import time
import logging
from typing import Tuple, List, Dict
import numpy as np
from scipy.fft import fft2, ifft2, fftshift, ifftshift

from config import SystemConfig

logger = logging.getLogger(__name__)


class FPMReconstructor:
    """
    Fourier Ptychography reconstruction engine.
    
    Implements iterative Gerchberg-Saxton algorithm with spectral
    constraint enforcement for phase retrieval.
    
    Attributes:
        config: System configuration object
        pupil_cache: Cached pupil functions for different image sizes
    """
    
    def __init__(self, config: SystemConfig):
        """
        Initialize reconstructor with system configuration.
        
        Args:
            config: SystemConfig instance with optical parameters
        """
        self.config = config
        self.pupil_cache = {}
        
        logger.info("FPM Reconstructor initialized")
        logger.info(f"  Wavelength: {config.optical.wavelength_nm} nm")
        logger.info(f"  Objective NA: {config.optical.na_objective}")
        logger.info(f"  Upsample factor: {config.reconstruction.upsample_factor}x")
        
    def _generate_pupil(self, shape: Tuple[int, int]) -> np.ndarray:
        """
        shape: LOW-RES image shape — pupil is generated in low-res pixel coordinates.
        The patch extraction in reconstruct() extracts a low_res_shape window from 
        the high_res spectrum, so pixel_size_m here must correspond to low-res pixels.
        This is CORRECT: effective_pixel_size = physical_pixel / magnification.
        No change needed — but document this explicitly to prevent future confusion.
        """
        cache_key = shape
        
        if cache_key in self.pupil_cache:
            return self.pupil_cache[cache_key]
        
        img_height, img_width = shape
        
        # Fourier frequency coordinates
        freq_x = np.fft.fftfreq(img_width, d=self.config.optical.pixel_size_m)
        freq_y = np.fft.fftfreq(img_height, d=self.config.optical.pixel_size_m)
        freq_xx, freq_yy = np.meshgrid(freq_x, freq_y)
        
        # Radial frequency in 1/m
        freq_radial = np.sqrt(freq_xx**2 + freq_yy**2)
        
        # Cutoff frequency from objective NA
        freq_max = self.config.optical.na_objective / self.config.optical.wavelength_m
        
        # Binary circular aperture
        pupil = (freq_radial <= freq_max).astype(np.complex64)
        
        self.pupil_cache[cache_key] = pupil
        
        logger.debug(f"Pupil function generated: {np.sum(pupil)} / {pupil.size} pixels active")
        
        return pupil
        
    def reconstruct(
        self,
        images: List[np.ndarray],
        kx_shifts: np.ndarray,
        ky_shifts: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """
        Execute Fourier ptychographic reconstruction.
        
        Iterative algorithm workflow:
        1. Initialize high-resolution spectrum with center LED image
        2. For each iteration:
            a. For each LED position:
                - Extract low-res region from high-res spectrum
                - Apply pupil function (spectral crop)
                - Inverse FFT to spatial domain
                - Replace amplitude with measured intensity (preserve phase)
                - Forward FFT back to Fourier domain
                - Update high-res spectrum in that region
        3. Repeat until convergence or max iterations
        4. Final inverse FFT to obtain complex-valued object
        
        Args:
            images: List of 25 captured intensity images (float32)
            kx_shifts: K-space x-shifts in pixel units (length 25)
            ky_shifts: K-space y-shifts in pixel units (length 25)
            
        Returns:
            Tuple of:
                - amplitude: High-resolution amplitude image (float32)
                - phase: High-resolution phase map in radians (float32)
                - metadata: Dictionary with convergence info
                
        Raises:
            ValueError: If input dimensions inconsistent
        """
        start_time = time.time()
        
        if len(images) != len(kx_shifts) or len(images) != len(ky_shifts):
            raise ValueError(
                f"Dimension mismatch: {len(images)} images but "
                f"{len(kx_shifts)} kx and {len(ky_shifts)} ky shifts"
            )
        
        low_res_shape = images[0].shape
        high_res_shape = (
            low_res_shape[0] * self.config.reconstruction.upsample_factor,
            low_res_shape[1] * self.config.reconstruction.upsample_factor
        )
        
        logger.info(f"Reconstruction started: {low_res_shape} → {high_res_shape}")
        logger.info(f"Input images: {len(images)}")
        
        # Normalize all images to [0,1] amplitude-squared space
        images_norm = [img / (np.max(img) + 1e-8) for img in images]

        low_res_shape = images_norm[0].shape
        high_res_shape = (
            low_res_shape[0] * self.config.reconstruction.upsample_factor,
            low_res_shape[1] * self.config.reconstruction.upsample_factor
        )
        
        pupil = self._generate_pupil(low_res_shape)
        pupil_conj = np.conj(pupil)
        pupil_max_sq = np.max(np.abs(pupil)**2) + 1e-8
        alpha = 0.8  # ePIE step size

        obj_spectrum = np.zeros(high_res_shape, dtype=np.complex64)
        
        center_y, center_x = high_res_shape[0] // 2, high_res_shape[1] // 2
        half_h, half_w = low_res_shape[0] // 2, low_res_shape[1] // 2
        
        center_image_spectrum = fftshift(fft2(np.sqrt(images_norm[12])))
        obj_spectrum[
            center_y - half_h:center_y + half_h,
            center_x - half_w:center_x + half_w
        ] = center_image_spectrum

        convergence_history = []
        converged = False

        for iteration in range(self.config.reconstruction.max_iterations):
            spectrum_old = obj_spectrum.copy()
            
            for led_idx in range(len(images_norm)):
                measured_intensity = images_norm[led_idx]
                shift_y = int(round(ky_shifts[led_idx]))
                shift_x = int(round(kx_shifts[led_idx]))

                y_start = center_y + shift_y - half_h
                x_start = center_x + shift_x - half_w
                y_end = y_start + low_res_shape[0]
                x_end = x_start + low_res_shape[1]

                if y_start < 0 or x_start < 0 or y_end > high_res_shape[0] or x_end > high_res_shape[1]:
                    logger.warning(f"LED {led_idx} k-shift OOB ({shift_y},{shift_x}), skipping")
                    continue

                # Extract patch and apply pupil
                patch = obj_spectrum[y_start:y_end, x_start:x_end] * pupil

                # Spatial estimate via IFFT
                spatial_estimate = ifft2(ifftshift(patch))

                # Amplitude constraint: replace magnitude, preserve phase
                amplitude_measured = np.sqrt(measured_intensity)
                phase_estimate = np.angle(spatial_estimate)
                spatial_updated = amplitude_measured * np.exp(1j * phase_estimate)

                # FFT back to Fourier domain
                spectrum_updated = fftshift(fft2(spatial_updated))

                # ePIE gradient update (replaces hard GS replacement)
                residual = spectrum_updated - patch
                obj_spectrum[y_start:y_end, x_start:x_end] += (
                    alpha * pupil_conj * residual / pupil_max_sq
                )

            # Convergence check
            relative_change = np.linalg.norm(obj_spectrum - spectrum_old) / (np.linalg.norm(spectrum_old) + 1e-8)
            convergence_history.append(float(relative_change))
            logger.info(f"Iter {iteration+1}: Δ={relative_change:.6f}")

            if relative_change < self.config.reconstruction.convergence_threshold:
                converged = True
                break

        reconstructed_complex = ifft2(ifftshift(obj_spectrum))
        amplitude = np.abs(reconstructed_complex).astype(np.float32)
        phase = np.angle(reconstructed_complex).astype(np.float32)

        return amplitude, phase, {
            "iterations_completed": iteration + 1,
            "converged": converged,
            "final_relative_change": convergence_history[-1],
            "convergence_history": convergence_history,
            "high_res_shape": high_res_shape,
            "upsample_factor": self.config.reconstruction.upsample_factor
        }