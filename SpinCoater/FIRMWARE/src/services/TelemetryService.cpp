#include "services/TelemetryService.h"
#include "common/configs.h"
#include <Arduino.h>

namespace Services {

    TelemetryService::TelemetryService(HAL::ISpinMotor& spin, HAL::IRpmSensor& tacho,
                                       HAL::IPumpMotor& pump, HAL::IZAxisMotor& zAxis,
                                       StateMachine::StateMachine& sm,
                                       ProcessOrchestrator& orchestrator)
        : sm_(sm), spin_(spin), sensor_(tacho), pump_(pump), zAxis_(zAxis),
          orchestrator_(orchestrator)
    {}

    void TelemetryService::update() {
        snapshot_.currentState = sm_.getCurrentState();

        snapshot_.targetSpinSpeed_rpm = spin_.getTargetSpeed();
        snapshot_.actualSpinSpeed_rpm = sensor_.readRPM();

        snapshot_.dispensedVolume_ml = pump_.getTotalDispensed_ml();
        snapshot_.pumpActive = pump_.isBusy();

        snapshot_.zPosition_mm = zAxis_.getCurrentPosition() / static_cast<float>(Z_STEPS_PER_MM);
        snapshot_.zAxisMoving = zAxis_.isMoving();

        snapshot_.currentStepIndex = orchestrator_.getCurrentStepIndex();
        snapshot_.totalSteps = orchestrator_.getTotalSteps();
        snapshot_.stepProgress_percent = orchestrator_.getStepProgress();
        snprintf(snapshot_.recipeName, sizeof(snapshot_.recipeName), "%s",
                 orchestrator_.getRecipeName());
        snprintf(snapshot_.phaseLabel, sizeof(snapshot_.phaseLabel), "%s",
                 orchestrator_.getPhaseLabel());

        snapshot_.lastUpdate_ms = millis();
    }
}
