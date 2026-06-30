#pragma once
#include "hal/interfaces/IZAxisMotor.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <HardwareSerial.h>
#include <Arduino.h>

namespace HAL {
    class TMC2209ZAxis : public IZAxisMotor {
        public:
            TMC2209ZAxis(HardwareSerial& serialPort, uint8_t stepPin, uint8_t dirPin,
                          uint8_t enablePin, uint8_t limitSwitchPin, float stepsPerMM);

            void begin();
            void moveTo(int32_t position) override;
            void home() override;
            void stop() override;
            void update() override;
            bool isMoving() const override;
            int32_t getCurrentPosition() const override;

            bool isHomed() const { return isHomed_; }

        private:
            HardwareSerial& serial_;
            TMC2209Stepper driver_;
            AccelStepper stepper_;
            uint8_t enablePin_;
            uint8_t limitSwitchPin_;
            float stepsPerMM_;
            bool isHomed_ = false;

            void homeBlocking();
    };
}
