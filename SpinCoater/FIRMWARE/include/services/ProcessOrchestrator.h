#pragma once
#include "domain/Recipe.h"
#include "domain/Event.h"
#include "controllers/SpinController.h"
#include "controllers/PumpController.h"
#include "controllers/ZAxisController.h"
#include <cstddef>
#include <functional>

namespace Services {

	/**
	 * Executes a recipe step-by-step.
	 *
	 * Driven into Running by the StateMachine (via Application). For each step it
	 * sequences the hardware:
	 *
	 *   Priming -> MovingToDispense -> Dispensing -> MovingToSpin
	 *           -> SpinningUp -> Holding -> SpinningDown -> (next step)
	 *
	 * Motion phases advance on controller completion (events) and are also
	 * polled as a backstop; timed phases advance on elapsed time. The
	 * orchestrator never mutates the StateMachine directly: it posts events
	 * (StepCompleted, ProcessCompleted, FaultDetected) back to it.
	 */
	class ProcessOrchestrator {
		public:
			using EventPoster = std::function<void(const Domain::Event&)>;

			ProcessOrchestrator(Controllers::SpinController& spin,
					    Controllers::PumpController& pump,
					    Controllers::ZAxisController& zAxis);

			void setEventPoster(EventPoster poster) { eventPoster_ = poster; }

			// Load (but do not start) a recipe.
			void loadRecipe(const Domain::Recipe& recipe);

			void start();
			void pause();
			void resume();
			void stop();

			// Advance timed phases and poll motion completion. Call frequently.
			void update();

			// Sink for controller feedback events (DispenseCompleted,
			// ZAxisMovementComplete, SpinSpeedReached).
			void onControllerEvent(const Domain::Event& event);

			// Read-only telemetry for TelemetryService.
			bool isActive() const;
			uint8_t getCurrentStepIndex() const;  // 1-based, 0 when idle
			uint8_t getTotalSteps() const;
			float getStepProgress() const { return stepProgress_; }
			const char* getRecipeName() const { return currentRecipe_.name; }
			const char* getPhaseLabel() const;

		private:
			enum class Phase : uint8_t {
				Idle,
				Priming,
				MovingToDispense,
				Dispensing,
				MovingToSpin,
				SpinningUp,
				Holding,
				SpinningDown,
				Completed
			};

			Controllers::SpinController& spin_;
			Controllers::PumpController& pump_;
			Controllers::ZAxisController& zAxis_;

			Domain::Recipe currentRecipe_;
			size_t stepIndex_ = 0;
			Phase phase_ = Phase::Idle;
			bool paused_ = false;

			unsigned long phaseStartMs_ = 0;
			unsigned long pauseStartMs_ = 0;
			float stepProgress_ = 0.0f;

			// Spindle ramp state (SpinningUp / SpinningDown).
			float rampFromRpm_ = 0.0f;
			float rampToRpm_ = 0.0f;
			unsigned long lastRampCmdMs_ = 0;

			EventPoster eventPoster_;

			const Domain::RecipeStep& currentStep() const;
			void beginStep();
			void enterPhase(Phase phase);
			void tryAdvance();
			void finishStep();

			// Linearly interpolate the spindle setpoint from..to over rampMs and
			// (throttled) command it. Returns true once the ramp duration elapsed.
			bool updateRamp(unsigned long now, float fromRpm, float toRpm, unsigned long rampMs);
			int32_t mmToSteps(float mm) const;
			static unsigned long secondsToMs(float seconds);
			void post(Domain::EventType type, int32_t param = 0);
	};
}
