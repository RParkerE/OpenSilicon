#include "hal/implementations/TMC2209ZAxis.h"
#include "common/configs.h"
#include "common/pin_mapping.h"
#include <Arduino.h>

namespace HAL {
    static const uint32_t HOMING_SPEED = 1000;

    TMC2209ZAxis::TMC2209ZAxis(HardwareSerial& serialPort, uint8_t stepPin, uint8_t dirPin,
                               uint8_t enablePin, uint8_t limitSwitchPin, float stepsPerMM)
        : serial_(serialPort),
          driver_(&serialPort, 0.11f, 0b00),
          stepper_(AccelStepper::DRIVER, stepPin, dirPin),
          enablePin_(enablePin),
          limitSwitchPin_(limitSwitchPin),
          stepsPerMM_(stepsPerMM)
    {}

    void TMC2209ZAxis::begin() {
        pinMode(limitSwitchPin_, INPUT_PULLUP);
        pinMode(enablePin_, OUTPUT);
        digitalWrite(enablePin_, LOW);

        serial_.begin(115200, SERIAL_8N1, Z_RX_PIN, Z_TX_PIN);
        driver_.begin();
        driver_.rms_current(800);
        driver_.microsteps(8);
        driver_.pwm_autoscale(true);

        stepper_.setMaxSpeed(2000);
        stepper_.setAcceleration(1000);

        homeBlocking();
    }

    void TMC2209ZAxis::homeBlocking() {
        isHomed_ = false;

        if (digitalRead(limitSwitchPin_) == LOW) {
            stepper_.setMaxSpeed(1000);
            stepper_.setAcceleration(500);
            stepper_.move(static_cast<long>(stepsPerMM_ * 10));
            while (stepper_.distanceToGo() != 0) {
                stepper_.run();
                delay(1);
            }
            delay(100);
        }

        stepper_.setMaxSpeed(HOMING_SPEED);
        stepper_.setAcceleration(500);
        stepper_.move(static_cast<long>(-stepsPerMM_ * 400));

        uint32_t homingStart = millis();
        while (digitalRead(limitSwitchPin_) == HIGH) {
            stepper_.run();

            if (millis() - homingStart > 30000) {
                stepper_.stop();
                Serial.println("Z homing timeout");
                return;
            }
            if (stepper_.distanceToGo() == 0) {
                Serial.println("Z homing failed - ran out of travel");
                return;
            }
            delay(1);
        }

        stepper_.stop();
        stepper_.setCurrentPosition(0);
        delay(100);

        stepper_.setMaxSpeed(1000);
        stepper_.setAcceleration(500);
        stepper_.moveTo(static_cast<long>(stepsPerMM_ * Z_HOMING_OFFSET_MM));
        while (stepper_.distanceToGo() != 0) {
            stepper_.run();
            delay(1);
        }

        stepper_.setCurrentPosition(0);
        stepper_.setMaxSpeed(2000);
        stepper_.setAcceleration(1000);
        isHomed_ = true;
    }

    void TMC2209ZAxis::moveTo(int32_t position) {
        if (!isHomed_) return;
        stepper_.moveTo(position);
    }

    void TMC2209ZAxis::home() {
        homeBlocking();
    }

    void TMC2209ZAxis::stop() {
        stepper_.stop();
        stepper_.setCurrentPosition(stepper_.currentPosition());
    }

    bool TMC2209ZAxis::isMoving() const {
        return const_cast<AccelStepper&>(stepper_).distanceToGo() != 0;
    }

    int32_t TMC2209ZAxis::getCurrentPosition() const {
        return static_cast<int32_t>(const_cast<AccelStepper&>(stepper_).currentPosition());
    }

    void TMC2209ZAxis::update() {
        stepper_.run();
    }
}
