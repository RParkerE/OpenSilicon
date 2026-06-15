For [Yaeonku BD015](https://www.amazon.com/dp/B0CPP2391F?th=1) OPU

| Pin(s) | Sub-System / Component | Measured Resistance | Status & Functional Notes |
|---|---|---|---|
| 1 | 405nm Laser Diode | N/A | Verified. Laser Cathode / Positive drive pin. Caution: Limit current to <80mA during tests. |
| 2, 3, 29 | Common Ground (GND) | N/A | Verified. Laser Common / System return path. |
| 16, 17, 18, 19 | Photodiode IC (PDIC) | 250Ω – 500Ω | Identified. Transimpedance amplifier outputs for the A, B, C, D astigmatic quadrants. Need to identify quadrant to pin. |
| 31, 32 | Voice Coil Motor (VCM A) | 4Ω | Identified. Low-impedance actuator. Action: Requires 45mA kinematic matrix test to determine if Focus, Tracking, or Tilt. |
| 33, 34 | Voice Coil Motor (VCM B) | 4Ω | Identified. Low-impedance actuator. Action: Requires 45mA kinematic matrix test to determine if Focus, Tracking, or Tilt. |
| 35, 36, 37, 38 | SA / Auxiliary Coil | 4Ω – 60Ω | Identified. Higher impedance indicates a secondary actuator, likely for Spherical Aberration (SA) correction or a micro-tilt element. |
