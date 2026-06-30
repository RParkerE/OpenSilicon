#pragma once
#include <cstdint>

namespace HAL {

	/**
	 * High-level interface for the spindle (ESC + brushless motor).
	 * Speed is commanded in RPM; actual speed is read back via IRpmSensor.
	 * The concrete driver maps RPM onto a DShot throttle (closed-loop in V1).
	 */
	class ISpinMotor {
		public:
			virtual ~ISpinMotor() = default;

			// Command the motor to maintain the given RPM (0 = stop).
			virtual void setTargetSpeed(int32_t rpm) = 0;

			// Stop immediately.
			virtual void stop() = 0;

			// Return the last commanded target speed (for telemetry).
			virtual int32_t getTargetSpeed() const = 0;
	};
}
