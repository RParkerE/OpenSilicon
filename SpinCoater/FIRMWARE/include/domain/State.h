#pragma once
#include <cstdint>

namespace Domain {

	/**
	 * High-level machine states.
	 * The StateMachine owns the current state and is the ONLY component allowed to change it.
	 */
	enum class MachineState : uint8_t {
		Idle,		// Ready, no process running
		Running,	// A recipe is being executed
		Paused,		// Recipe execution temporarily suspended
		Fault,		// Unrecoverable error, requires manual reset
		Cleaning,	// Optional maintenance cycle
		Unknown		// Initial state before first transition
	};
}
