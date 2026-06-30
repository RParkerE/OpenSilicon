#pragma once
#include "domain/RuntimeTelemetry.h"
#include "state_machine/StateMachine.h"
#include "services/ProcessOrchestrator.h"
#include "hal/interfaces/ISpinMotor.h"
#include "hal/interfaces/IRpmSensor.h"
#include "hal/interfaces/IPumpMotor.h"
#include "hal/interfaces/IZAxisMotor.h"

namespace Services {

	/**
	 * Collects live data from the HAL and the ProcessOrchestrator and builds a
	 * single RuntimeTelemetry snapshot for the DisplayService and WebSocket.
	 *
	 * Follows the rule: the display never reads hardware/controllers directly.
	 */
	class TelemetryService {
		public:
			TelemetryService(HAL::ISpinMotor& spin, HAL::IRpmSensor& tacho,
					 HAL::IPumpMotor& pump, HAL::IZAxisMotor& zAxis,
					 StateMachine::StateMachine& sm,
					 ProcessOrchestrator& orchestrator);

			// Gather the latest data into the snapshot. Call regularly.
			void update();

			// Most recent snapshot, consumed by DisplayService and WebSocket.
			const Domain::RuntimeTelemetry& getSnapshot() const { return snapshot_; }

		private:
			StateMachine::StateMachine& sm_;
			HAL::ISpinMotor& spin_;
			HAL::IRpmSensor& sensor_;
			HAL::IPumpMotor& pump_;
			HAL::IZAxisMotor& zAxis_;
			ProcessOrchestrator& orchestrator_;

			Domain::RuntimeTelemetry snapshot_;
	};
}
