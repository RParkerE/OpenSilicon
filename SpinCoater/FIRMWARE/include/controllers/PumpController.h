#pragma once
#include "hal/interfaces/IPumpMotor.h"
#include "domain/Event.h"
#include <functional>

namespace Controllers {

	/**
	 * Syringe-pump controller.
	 *
	 * Converts high-level dispense/aspirate commands into motor actions,
	 * services the motor in update(), and emits a DispenseCompleted event once
	 * the requested volume has been delivered (or withdrawn).
	 */
	class PumpController {
		public:
			using EventCallback = std::function<void(const Domain::Event&)>;

			explicit PumpController(HAL::IPumpMotor& motor) : motor_(motor) {}

			void setEventCallback(EventCallback cb) { eventCallback_ = cb; }

			// Start dispensing the given volume (non-blocking).
			void dispense(float volume_ml);

			// Start aspirating (withdrawing) the given volume, e.g. to prime.
			void aspirate(float volume_ml);

			void stop();

			// Service motion and emit completion events. Call frequently.
			void update();

			float getTotalDispensed_ml() const { return motor_.getTotalDispensed_ml(); }
			bool isBusy() const { return motor_.isBusy(); }

		private:
			HAL::IPumpMotor& motor_;
			EventCallback eventCallback_;
			bool operationInProgress_ = false;

			void emit(Domain::EventType type, int32_t param = 0);
	};
}
