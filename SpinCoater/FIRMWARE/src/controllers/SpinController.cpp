#include "controllers/SpinController.h"
#include "common/configs.h"
#include <cstdlib>

namespace Controllers {

	void SpinController::emit(Domain::EventType type, int32_t param) {
		if (eventCallback_) {
			eventCallback_({type, param});
		}
	}

	void SpinController::setTargetSpeed(int32_t rpm) {
		if (rpm < 0) rpm = 0;
		targetRPM_ = rpm;
		speedReached_ = false;
		motor_.setTargetSpeed(rpm);
	}

	void SpinController::stop() {
		targetRPM_ = 0;
		speedReached_ = false;
		motor_.stop();
	}

	void SpinController::update() {
		actualRPM_ = sensor_.readRPM();

		if (targetRPM_ > 0 && !speedReached_) {
			if (std::abs(actualRPM_ - targetRPM_) <= SPIN_SPEED_TOLERANCE_RPM) {
				speedReached_ = true;
				emit(Domain::EventType::SpinSpeedReached, actualRPM_);
			}
		}
	}
}
