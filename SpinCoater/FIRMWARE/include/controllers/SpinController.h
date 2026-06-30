#pragma once
#include "hal/interfaces/ISpinMotor.h"
#include "hal/interfaces/IRpmSensor.h"
#include "domain/Event.h"
#include <functional>

namespace Controllers {

	/**
	 * Spindle (ESC + brushless motor) controller.
	 *
	 * Closed-loop speed regulation lives in the ESC HAL; this controller
	 * commands a target speed, monitors the RPM sensor in update(), and emits
	 * SpinSpeedReached once the measured speed settles within tolerance of a
	 * non-zero target. Never touches the StateMachine directly.
	 */
	class SpinController {
		public:
			using EventCallback = std::function<void(const Domain::Event&)>;

			SpinController(HAL::ISpinMotor& motor, HAL::IRpmSensor& sensor)
				: motor_(motor), sensor_(sensor) {}

			void setEventCallback(EventCallback cb) { eventCallback_ = cb; }

			// Command a new target speed in RPM (0 = stop).
			void setTargetSpeed(int32_t rpm);

			void stop();

			// Read the sensor, emit SpinSpeedReached when stable. Call frequently.
			void update();

			int32_t getTargetSpeed() const { return targetRPM_; }
			int32_t getActualSpeed() const { return actualRPM_; }

		private:
			HAL::ISpinMotor& motor_;
			HAL::IRpmSensor& sensor_;
			EventCallback eventCallback_;

			int32_t targetRPM_ = 0;
			int32_t actualRPM_ = 0;
			bool speedReached_ = false;

			void emit(Domain::EventType type, int32_t param = 0);
	};
}
