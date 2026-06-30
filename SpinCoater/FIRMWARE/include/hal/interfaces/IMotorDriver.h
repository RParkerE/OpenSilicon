#pragma once

namespace HAL {
	/**
	 * Base interface for any stepper motor driver
	 */
	class IMotorDriver {
		public:
			virtual ~IMotorDriver() = default;

			// Enable/disable motor power
			virtual void enable(bool on) = 0;

			// Set movement speed in steps per second and direction
			virtual void setSpeed(float stepsPerSecond, bool forward) = 0;

			// Stop immediatly (may decelerate)
			virtual void stop() = 0;

			// Return current position in steps (0 after homing)
			virtual int32_t getCurrentPositionSteps() = 0;

			// Move to absolute step position
			virtual void moveTo(int32_t targetSteps) = 0;

			// True while motor is moving
			virtual bool isMoving() const = 0;
	};
}
