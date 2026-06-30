#include "controllers/PumpController.h"

namespace Controllers {

	void PumpController::emit(Domain::EventType type, int32_t param) {
		if (eventCallback_) {
			eventCallback_({type, param});
		}
	}

	void PumpController::dispense(float volume_ml) {
		motor_.dispense(volume_ml);
		operationInProgress_ = true;
	}

	void PumpController::aspirate(float volume_ml) {
		motor_.aspirate(volume_ml);
		operationInProgress_ = true;
	}

	void PumpController::stop() {
		motor_.stop();
		operationInProgress_ = false;
	}

	void PumpController::update() {
		motor_.update();

		if (operationInProgress_ && !motor_.isBusy()) {
			operationInProgress_ = false;
			emit(Domain::EventType::DispenseCompleted);
		}
	}
}
