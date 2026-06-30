#include "services/DisplayService.h"
#include "common/configs.h"
#include "hal/interfaces/IDisplay.h"
#include <Arduino.h>
#include <cmath>
#include <cstring>

namespace Services {
    DisplayService::DisplayService(StateMachine::StateMachine& sm, HAL::IDisplay& display)
        : sm_(sm), display_(display)
    {
        memset(&model_, 0, sizeof(model_));
        strncpy(model_.statusMessage, "Idle", sizeof(model_.statusMessage) - 1);
    }

    bool DisplayService::modelChanged(const Domain::DisplayModel& model) const {
        if (model.activePage != lastRendered_.activePage) {
            return true;
        }
        if (strcmp(model.statusMessage, lastRendered_.statusMessage) != 0 ||
            strcmp(model.recipeName, lastRendered_.recipeName) != 0) {
            return true;
        }
        if (model.actualSpeed != lastRendered_.actualSpeed ||
            model.targetSpeed != lastRendered_.targetSpeed) {
            return true;
        }
        if (model.currentStep != lastRendered_.currentStep ||
            model.totalSteps != lastRendered_.totalSteps) {
            return true;
        }
        if (model.activePage == Domain::DisplayPage::Fault) {
            if (model.faultCode != lastRendered_.faultCode ||
                strcmp(model.faultDescription, lastRendered_.faultDescription) != 0) {
                return true;
            }
        }
        return millis() - lastRenderMs_ >= DISPLAY_REFRESH_MS;
    }

    void DisplayService::update(const Domain::RuntimeTelemetry& telemetry,
                               const Domain::Recipe* /*activeRecipe*/) {
        model_.activePage = pageForState(sm_.getCurrentState());

        // The simple status screen shows the same live fields on every page.
        snprintf(model_.statusMessage, sizeof(model_.statusMessage), "%s",
                 telemetry.phaseLabel[0] ? telemetry.phaseLabel : "Idle");
        snprintf(model_.recipeName, sizeof(model_.recipeName), "%s", telemetry.recipeName);
        model_.targetSpeed = telemetry.targetSpinSpeed_rpm;
        model_.actualSpeed = telemetry.actualSpinSpeed_rpm;
        model_.currentStep = telemetry.currentStepIndex;
        model_.totalSteps = telemetry.totalSteps;

        if (model_.activePage == Domain::DisplayPage::Fault) {
            buildFaultPage(telemetry);
        }

        if (!modelChanged(model_)) {
            return;
        }

        display_.render(model_);
        lastRendered_ = model_;
        lastRenderMs_ = millis();
    }

    void DisplayService::showPage(Domain::DisplayPage page) {
        model_.activePage = page;
    }

    Domain::DisplayPage DisplayService::pageForState(Domain::MachineState state) {
        return state == Domain::MachineState::Fault ? Domain::DisplayPage::Fault
                                                    : Domain::DisplayPage::Status;
    }

    void DisplayService::buildFaultPage(const Domain::RuntimeTelemetry& t) {
        model_.faultCode = t.faultCode;
        strncpy(model_.faultDescription, t.faultDescription, sizeof(model_.faultDescription) - 1);
    }
}
