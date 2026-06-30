#include "hal/implementations/Pump28BYJ48.h"

namespace HAL {
    Pump28BYJ48::Pump28BYJ48(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, float stepsPerML)
        : stepper_(AccelStepper::HALF4WIRE, in1, in2, in3, in4),
          stepsPerML_(stepsPerML),
          totalDispensed_(0.0f),
          busy_(false)
    {}

    void Pump28BYJ48::begin() {
        stepper_.setMaxSpeed(1000);
        stepper_.setAcceleration(200);
        stepper_.setCurrentPosition(0);
    }

    void Pump28BYJ48::dispense(float volume_ml) {
        long steps = volume_ml * stepsPerML_;
        stepper_.move(steps);
        busy_ = true;
    }

    void Pump28BYJ48::aspirate(float volume_ml) {
        long steps = -volume_ml * stepsPerML_;
        stepper_.move(steps);
        busy_ = true;
    }

    void Pump28BYJ48::stop() {
        stepper_.stop();
        busy_ = false;
    }

    bool Pump28BYJ48::isBusy() const {
        return stepper_.distanceToGo() != 0;
    }

    float Pump28BYJ48::getTotalDispensed_ml() const {
        return totalDispensed_;
    }

    void Pump28BYJ48::update() {
        if (busy_) {
            stepper_.run();
            if (!stepper_.isRunning()) {
                long moved = stepper_.currentPosition();
                totalDispensed_ += moved / stepsPerML_;
                stepper_.setCurrentPosition(0);
                busy_ = false;
            }
        }
    }
}
