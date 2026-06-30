#include "controllers/ZAxisController.h"

namespace Controllers {

	void ZAxisController::emit(Domain::EventType type, int32_t param) {
		if (eventCallback_) {
			eventCallback_({type, param});
		}
	}

	void ZAxisController::home() {
		motor_.home();
	}

	void ZAxisController::moveTo(int32_t positionSteps) {
		motor_.moveTo(positionSteps);
		moveInProgress_ = true;
	}

	void ZAxisController::stop() {
		motor_.stop();
		moveInProgress_ = false;
	}

	void ZAxisController::update() {
		motor_.update();

		// Fire completion once the commanded move finishes. A zero-distance move
		// completes on the next update (isMoving() is already false), so callers
		// always receive exactly one event.
		if (moveInProgress_ && !motor_.isMoving()) {
			moveInProgress_ = false;
			emit(Domain::EventType::ZAxisMovementComplete, motor_.getCurrentPosition());
		}
	}
}
