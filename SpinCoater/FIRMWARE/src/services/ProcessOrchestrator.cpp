#include "services/ProcessOrchestrator.h"
#include "common/configs.h"
#include <Arduino.h>

namespace Services {

    ProcessOrchestrator::ProcessOrchestrator(Controllers::SpinController& spin,
                                             Controllers::PumpController& pump,
                                             Controllers::ZAxisController& zAxis)
        : spin_(spin), pump_(pump), zAxis_(zAxis)
    {}

    void ProcessOrchestrator::post(Domain::EventType type, int32_t param) {
        if (eventPoster_) {
            eventPoster_({type, param});
        }
    }

    unsigned long ProcessOrchestrator::secondsToMs(float seconds) {
        if (seconds < 0.0f) seconds = 0.0f;
        return static_cast<unsigned long>(seconds * 1000.0f);
    }

    int32_t ProcessOrchestrator::mmToSteps(float mm) const {
        if (mm < 0.0f) mm = 0.0f;
        if (mm > Z_MAX_TRAVEL_MM) mm = Z_MAX_TRAVEL_MM;
        return static_cast<int32_t>(mm * Z_STEPS_PER_MM);
    }

    const Domain::RecipeStep& ProcessOrchestrator::currentStep() const {
        return currentRecipe_.steps[stepIndex_];
    }

    bool ProcessOrchestrator::isActive() const {
        return phase_ != Phase::Idle && phase_ != Phase::Completed;
    }

    uint8_t ProcessOrchestrator::getCurrentStepIndex() const {
        return isActive() ? static_cast<uint8_t>(stepIndex_ + 1) : 0;
    }

    uint8_t ProcessOrchestrator::getTotalSteps() const {
        return static_cast<uint8_t>(currentRecipe_.steps.size());
    }

    const char* ProcessOrchestrator::getPhaseLabel() const {
        if (paused_) return "Paused";
        switch (phase_) {
            case Phase::Priming:          return "Priming";
            case Phase::MovingToDispense: return "Moving Z";
            case Phase::Dispensing:       return "Dispensing";
            case Phase::MovingToSpin:     return "Moving Z";
            case Phase::SpinningUp:       return "Ramp Up";
            case Phase::Holding:          return "Spinning";
            case Phase::SpinningDown:     return "Ramp Down";
            case Phase::Completed:        return "Done";
            default:                      return "Idle";
        }
    }

    bool ProcessOrchestrator::updateRamp(unsigned long now, float fromRpm,
                                         float toRpm, unsigned long rampMs) {
        float frac = (rampMs == 0) ? 1.0f
                                   : static_cast<float>(now - phaseStartMs_) / rampMs;
        if (frac > 1.0f) frac = 1.0f;
        if (frac < 0.0f) frac = 0.0f;

        if (now - lastRampCmdMs_ >= RAMP_CMD_INTERVAL_MS) {
            lastRampCmdMs_ = now;
            const int32_t setpoint =
                static_cast<int32_t>(fromRpm + (toRpm - fromRpm) * frac);
            spin_.setTargetSpeed(setpoint);
        }
        return now - phaseStartMs_ >= rampMs;
    }

    void ProcessOrchestrator::loadRecipe(const Domain::Recipe& recipe) {
        currentRecipe_ = recipe;
        stepIndex_ = 0;
        phase_ = Phase::Idle;
        paused_ = false;
        stepProgress_ = 0.0f;
    }

    void ProcessOrchestrator::start() {
        if (currentRecipe_.steps.empty()) {
            post(Domain::EventType::ProcessCompleted);
            return;
        }
        stepIndex_ = 0;
        paused_ = false;
        beginStep();
    }

    void ProcessOrchestrator::enterPhase(Phase phase) {
        phase_ = phase;
        phaseStartMs_ = millis();
    }

    void ProcessOrchestrator::beginStep() {
        stepProgress_ = 0.0f;
        const Domain::RecipeStep& step = currentStep();

        if (step.primeVolume_ml > 0.0f) {
            enterPhase(Phase::Priming);
            pump_.aspirate(step.primeVolume_ml);
        } else {
            enterPhase(Phase::MovingToDispense);
            zAxis_.moveTo(mmToSteps(step.dispenseHeight_mm));
        }
    }

    void ProcessOrchestrator::onControllerEvent(const Domain::Event& /*event*/) {
        // Controller feedback simply nudges the state machine; tryAdvance()
        // re-reads the (now-updated) controller state to decide the transition.
        tryAdvance();
    }

    void ProcessOrchestrator::update() {
        tryAdvance();
    }

    void ProcessOrchestrator::tryAdvance() {
        if (paused_ || phase_ == Phase::Idle || phase_ == Phase::Completed) {
            return;
        }

        const Domain::RecipeStep& step = currentStep();
        const unsigned long now = millis();

        switch (phase_) {
            case Phase::Priming:
                if (!pump_.isBusy()) {
                    enterPhase(Phase::MovingToDispense);
                    zAxis_.moveTo(mmToSteps(step.dispenseHeight_mm));
                }
                break;

            case Phase::MovingToDispense:
                if (!zAxis_.isMoving()) {
                    if (step.dispenseVolume_ml > 0.0f) {
                        enterPhase(Phase::Dispensing);
                        pump_.dispense(step.dispenseVolume_ml);
                    } else {
                        enterPhase(Phase::MovingToSpin);
                        zAxis_.moveTo(mmToSteps(step.spinHeight_mm));
                    }
                }
                break;

            case Phase::Dispensing:
                if (!pump_.isBusy()) {
                    enterPhase(Phase::MovingToSpin);
                    zAxis_.moveTo(mmToSteps(step.spinHeight_mm));
                }
                break;

            case Phase::MovingToSpin:
                if (!zAxis_.isMoving()) {
                    enterPhase(Phase::SpinningUp);
                    rampFromRpm_ = static_cast<float>(spin_.getActualSpeed());
                    rampToRpm_ = step.spinSpeed_rpm;
                    lastRampCmdMs_ = 0;
                }
                break;

            case Phase::SpinningUp: {
                const unsigned long rampMs = secondsToMs(step.rampTime_s);
                const bool done = updateRamp(now, rampFromRpm_, rampToRpm_, rampMs);
                stepProgress_ = (rampMs == 0)
                    ? 100.0f
                    : static_cast<float>(now - phaseStartMs_) / rampMs * 100.0f;
                if (stepProgress_ > 100.0f) stepProgress_ = 100.0f;
                if (done) {
                    spin_.setTargetSpeed(static_cast<int32_t>(rampToRpm_));
                    enterPhase(Phase::Holding);
                    stepProgress_ = 0.0f;
                }
                break;
            }

            case Phase::Holding: {
                const unsigned long hold = secondsToMs(step.holdTime_s);
                stepProgress_ = (hold == 0)
                    ? 100.0f
                    : static_cast<float>(now - phaseStartMs_) / hold * 100.0f;
                if (stepProgress_ > 100.0f) stepProgress_ = 100.0f;

                if (now - phaseStartMs_ >= hold) {
                    enterPhase(Phase::SpinningDown);
                    rampFromRpm_ = static_cast<float>(spin_.getActualSpeed());
                    rampToRpm_ = 0.0f;
                    lastRampCmdMs_ = 0;
                }
                break;
            }

            case Phase::SpinningDown: {
                const unsigned long rampMs = secondsToMs(step.rampTime_s);
                const bool done = updateRamp(now, rampFromRpm_, rampToRpm_, rampMs);
                if (done) {
                    spin_.setTargetSpeed(0);
                    finishStep();
                }
                break;
            }

            default:
                break;
        }
    }

    void ProcessOrchestrator::finishStep() {
        stepIndex_++;
        if (stepIndex_ >= currentRecipe_.steps.size()) {
            phase_ = Phase::Completed;
            spin_.stop();
            post(Domain::EventType::ProcessCompleted);
        } else {
            post(Domain::EventType::StepCompleted, static_cast<int32_t>(stepIndex_));
            beginStep();
        }
    }

    void ProcessOrchestrator::pause() {
        if (!isActive() || paused_) {
            return;
        }
        paused_ = true;
        pauseStartMs_ = millis();
        spin_.setTargetSpeed(0);  // stop the spindle while paused
    }

    void ProcessOrchestrator::resume() {
        if (!paused_) {
            return;
        }
        paused_ = false;
        // Shift the phase clock so timed phases resume where they left off.
        phaseStartMs_ += millis() - pauseStartMs_;
        lastRampCmdMs_ = 0;  // force an immediate setpoint re-command

        // Holding resumes at full speed; ramp phases pick back up via updateRamp.
        if (phase_ == Phase::Holding) {
            spin_.setTargetSpeed(static_cast<int32_t>(currentStep().spinSpeed_rpm));
        }
    }

    void ProcessOrchestrator::stop() {
        phase_ = Phase::Idle;
        paused_ = false;
        stepIndex_ = 0;
        stepProgress_ = 0.0f;
        spin_.stop();
        pump_.stop();
        zAxis_.stop();
    }
}
