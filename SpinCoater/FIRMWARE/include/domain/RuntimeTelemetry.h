#pragma once
#include "State.h"
#include <cstdint>

namespace Domain {

	/**
	 * Snapshot of all live data needed by the display and websocket.
	 * Populated exclusively by TelemetryService from the HAL and ProcessOrchestrator.
	 * DisplayService only reads this structure, never the hardware directly.
	 */
	struct RuntimeTelemetry {
		MachineState currentState = MachineState::Unknown;

		// Spin motor data
		int32_t targetSpinSpeed_rpm = 0;
		int32_t actualSpinSpeed_rpm = 0;	// From ESC in V1, Hall sensors to be added in V2

		// Pump data
		float dispensedVolume_ml = 0.0f;
		bool pumpActive = false;

		// Z-axis data
		float zPosition_mm = 0.0f;
		bool zAxisMoving = false;

		// Process Info
		char recipeName[32] = {0};
		char phaseLabel[16] = {0};		// Human-readable process phase
		uint8_t currentStepIndex = 0;
		uint8_t totalSteps = 0;
		float stepProgress_percent = 0.0f;	// 0-100% completion of current steps

		// Fault info
		uint16_t faultCode = 0;
		char faultDescription[64] = {0};

		// Timestamp for display refresh
		uint32_t lastUpdate_ms = 0;
	};
}
