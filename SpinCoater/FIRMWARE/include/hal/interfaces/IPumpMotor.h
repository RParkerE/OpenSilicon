#pragma once

namespace HAL {

	/**
	 * Syringe pump interface (28BYJ-48 stepper motor).
	 * All volume commands are in millilitres; the implementation converts to steps.
	 */
	class IPumpMotor {
		public:
			virtual ~IPumpMotor() = default;

			// Dispense a given volume in millilitres
			virtual void dispense(float volume_ml) = 0;

			// Aspirate (withdraw) a given volume
			virtual void aspirate(float volume_ml) = 0;

			// Stop all movement
			virtual void stop() = 0;

			// Service the motor (run the step generator). Call frequently.
			virtual void update() = 0;

			// Check if a dispense/aspirate operation is ongoing
			virtual bool isBusy() const = 0;

			// Get total dispensed volume since last reset (for telemetry)
			virtual float getTotalDispensed_ml() const = 0;
	};
}
