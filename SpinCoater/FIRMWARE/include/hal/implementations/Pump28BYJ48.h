#pragma once
#include "hal/interfaces/IPumpMotor.h"
#include <AccelStepper.h>

namespace HAL {
    class Pump28BYJ48 : public IPumpMotor {
        public:
            /**
             * @param in1, in2, in3, in4    Pin numbers for the 4 motor coils (ULN2003 inputs)
             * @param stepsPerML            Calibrated motor steps per millilitre of fluid
             */
            Pump28BYJ48(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, float stepsPerML);
            void begin();

            // IPumpMotor interface
            void dispense(float volume_ml) override;
            void aspirate(float volume_ml) override;
            void stop() override;
            void update() override;
            bool isBusy() const override;
            float getTotalDispensed_ml() const override;

        private:
            mutable AccelStepper stepper_;
            float stepsPerML_;
            float totalDispensed_;
            bool busy_;
    };
}
