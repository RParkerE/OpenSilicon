#pragma once
#include "hal/interfaces/IZAxisMotor.h"
#include "domain/Event.h"
#include <functional>

namespace Controllers {

	/**
	 * Z-axis positioning controller.
	 *
	 * Wraps the Z-axis motor HAL, services its motion in update(), and emits a
	 * ZAxisMovementComplete event exactly once when a commanded move finishes
	 * (including zero-distance moves, so callers never stall waiting for an
	 * event that would otherwise never arrive).
	 */
	class ZAxisController {
		public:
			using EventCallback = std::function<void(const Domain::Event&)>;

			explicit ZAxisController(HAL::IZAxisMotor& motor) : motor_(motor) {}

			void setEventCallback(EventCallback cb) { eventCallback_ = cb; }

			// Run the homing sequence (blocking in the HAL).
			void home();

			// Move to an absolute position in motor steps.
			void moveTo(int32_t positionSteps);

			void stop();

			// Service motion and emit completion events. Call frequently.
			void update();

			int32_t getCurrentPosition() const { return motor_.getCurrentPosition(); }
			bool isMoving() const { return motor_.isMoving(); }

		private:
			HAL::IZAxisMotor& motor_;
			EventCallback eventCallback_;
			bool moveInProgress_ = false;

			void emit(Domain::EventType type, int32_t param = 0);
	};
}
