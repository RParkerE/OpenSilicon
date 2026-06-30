#pragma once
#include <cstdint>

namespace HAL {

	/**
	 * Z-axis linear actuator (TMC2209-driven NEMA17 stepper)
	 * Responsible for raising/lowering the dispensing head.
	 * Position is expressed in micrometers (or mm, as chosen).
	 */
	class IZAxisMotor {
		public:
			virtual ~IZAxisMotor() = default;

			// Move to absolute position in steps (or user units after conversion)
			virtual void moveTo(int32_t position) = 0;

			// Perform homing sequence (limit switch / stall detection)
			virtual void home() = 0;

			// Stop immediately
			virtual void stop() = 0;

			// Service the motor (run the step generator). Call frequently.
			virtual void update() = 0;

			// True while motor is moving
			virtual bool isMoving() const = 0;

			// Current position in the same units as moveTo
			virtual int32_t getCurrentPosition() const = 0;
	};
}
