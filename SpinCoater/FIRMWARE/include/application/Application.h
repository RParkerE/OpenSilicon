#pragma once
#include "hal/implementations/TFTDisplayHAL.h"
#include "hal/implementations/TMC2209ZAxis.h"
#include "hal/implementations/Pump28BYJ48.h"
#include "hal/implementations/EscSpinMotor.h"
#include "controllers/SpinController.h"
#include "controllers/PumpController.h"
#include "controllers/ZAxisController.h"
#include "services/DisplayService.h"
#include "services/TelemetryService.h"
#include "services/ProcessOrchestrator.h"
#include "SimpleRecipeManager.h"
#include "network/WebSocketServer.h"
#include "state_machine/StateMachine.h"
#include "domain/RuntimeTelemetry.h"

namespace Application {

	/**
	 * Top-level application object.
	 *
	 * Owns every component, wires them together (HAL -> Controllers ->
	 * ProcessOrchestrator -> StateMachine, plus the Display/Telemetry/WebSocket
	 * services), and runs the cooperative main loop. It contains no process
	 * sequencing logic itself - that lives in the ProcessOrchestrator.
	 */
	class Application {
		public:
			Application();
			void setup();
			void loop();

		private:
			// --- HAL ---
			TFT_eSPI tft_;
			HAL::TFTDisplayHAL displayHAL_;
			HardwareSerial tmcSerial_{2};
			HAL::EscSpinMotor spinMotor_;
			HAL::TMC2209ZAxis zAxis_;
			HAL::Pump28BYJ48 pump_;

			// --- Controllers ---
			Controllers::SpinController spinController_;
			Controllers::PumpController pumpController_;
			Controllers::ZAxisController zAxisController_;

			// --- Domain / services ---
			SimpleRecipeManager recipeManager_;
			StateMachine::StateMachine stateMachine_;
			Services::ProcessOrchestrator orchestrator_;
			Services::TelemetryService telemetryService_;
			Services::DisplayService displayService_;
			Network::WebSocketServer webSocket_;

			void setupEventRouting();
			void refreshDisplay();
			static void displayTaskEntry(void* param);
	};
}
