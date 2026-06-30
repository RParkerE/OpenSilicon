#define PUMP_STEPS_PER_ML   10   // Calibrate for this value
#define Z_STEPS_PER_MM      200  // Matches working demo (steps_per_mm)
#define Z_HOMING_OFFSET_MM  5.0f
#define Z_MAX_TRAVEL_MM     150  // Soft limit used to validate recipe heights

// Spin speed is considered "reached" when |actual - target| is within this band.
#define SPIN_SPEED_TOLERANCE_RPM  200

// How often the orchestrator re-commands the spindle setpoint while ramping
// (ms). Fine enough to look smooth, coarse enough not to flood the ESC queue.
#define RAMP_CMD_INTERVAL_MS  50

#define MOTOR_POLES         14
#define MOTOR_POLE_PAIRS    7
#define MAX_RPM             8000

// DShot protocol/timing is owned by the ESP32_DSHOT library. We run
// DSHOT300_BIDIR (bidirectional): a single signal wire carries the throttle
// out and the GCR-encoded eRPM telemetry back. DShot1200 bidir is unreliable
// on the ESP32, so 300 is used. DSHOT_SPEED_KBPS is retained only so the
// EscSpinMotor constructor signature stays stable; it is otherwise ignored.
#define DSHOT_SPEED_KBPS    300

// Motor control / telemetry task interval (ms)
#define ESC_UPDATE_INTERVAL_MS  10

#define TELEMETRY_BROADCAST_INTERVAL_MS  50
#define DISPLAY_REFRESH_MS               500

#define RECIPE_MIN_PHASE_MS              1000
