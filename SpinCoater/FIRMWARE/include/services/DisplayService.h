#pragma once
#include "domain/DisplayModel.h"
#include "domain/RuntimeTelemetry.h"
#include "state_machine/StateMachine.h"
#include "hal/interfaces/IDisplay.h"

namespace Services {

	/**
	 * Converts telemetry and machine state into a DisplayModel,
	 * then pushes it to the display HAL.
	 * Guarantees: Never reads hardware or controllers directly.
	 */
	class DisplayService {
		public:
			DisplayService(StateMachine::StateMachine& sm, HAL::IDisplay& display);

			// Call whenever telemetry or state changes.
			// Responsible for updating the model and trigger rendering.
			void update(const Domain::RuntimeTelemetry& telemetry,
				    const Domain::Recipe* activeRecipe = nullptr);

			// Manually force a page
			void showPage(Domain::DisplayPage page);

		private:
			StateMachine::StateMachine& sm_;
			HAL::IDisplay& display_;
			Domain::DisplayModel model_;
			Domain::DisplayModel lastRendered_;
			uint32_t lastRenderMs_ = 0;

			bool modelChanged(const Domain::DisplayModel& model) const;

			void buildFaultPage(const Domain::RuntimeTelemetry& telemetry);
			Domain::DisplayPage pageForState(Domain::MachineState state);
	};
}
