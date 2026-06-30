#pragma once
#include <cstdint>

namespace Domain {
	
	/**
	 * Event types used to trigger state transitions.
	 * Controllers, WebSocket, and the ProcessOrchestrator emit these events.
	 * The StateMachine consumes them and decides on transitions.
	 */
	enum class EventType : uint8_t {
		// User commands (from WebSocket)
		StartRequested,
		PauseRequested,
		ResumeRequested,
		StopRequested,
		ResetRequested,

		// Process orchestration events
		StepCompleted,
		ProcessCompleted,
		FaultDetected,

		// Controller feedback events
		SpinSpeedReached,
		DispenseCompleted,
		ZAxisMovementComplete,

		// Hardware fault events
		MotorFault,
		OverTemperature,
		EmergencyStop
	};

	/**
	 * Generic event carrying a type and optional parameter.
	 */
	struct Event {
		EventType type;
		int32_t param = 0;	// e.g., step index, error code

		Event(EventType t, int32_t p = 0) : type(t), param(p) {}
		Event() = default;
	};
}
