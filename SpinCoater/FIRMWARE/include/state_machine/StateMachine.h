#pragma once
#include "domain/State.h"
#include "domain/Event.h"
#include <queue>
#include <functional>

namespace StateMachine {
	
	/**
	 * The sole authority for state transitions.
	 * All components post events; only this class performs transitions.
	 * No HAL or Display code ever calls this class directly (except for events).
	 */
	class StateMachine {
		public:
			StateMachine();

			// Thread-safe event injection. Called by WebSocket, Controllers, etc.
			void postEvent(const Domain::Event& event);

			// Must be called frequently to process events and execute transitions
			void update();

			// Current machine state (read-only for DisplayService and ProcessOrchestrator
			Domain::MachineState getCurrentState() const;

			// Callback invoked whenever the state changes.
			// Used by ProcessOrchestrator to start/stop recipe execution.
			using StateChangeCallback = std::function<void(Domain::MachineState newState)>;
			void setStateChangeCallback(StateChangeCallback cb);

		private:
			Domain::MachineState currentState_ = Domain::MachineState::Unknown;
			std::queue<Domain::Event> eventQueue_;
			StateChangeCallback stateChangeCallback_;

			// Pure state transition logic (no side effects)
			Domain::MachineState determineNextState(Domain::MachineState current,
								const Domain::Event& event);
			void applyTransition(Domain::MachineState newState);
	};
}
