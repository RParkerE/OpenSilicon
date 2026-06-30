#include "application/Application.h"
#include "common/pin_mapping.h"
#include "common/configs.h"
#include <LittleFS.h>
#include "esp_task_wdt.h"

namespace Application {

	namespace {
		SemaphoreHandle_t displayMutex = nullptr;
	}

	Application::Application()
		: displayHAL_(tft_),
		  spinMotor_((gpio_num_t)ESC_PWM_PIN, DSHOT_SPEED_KBPS, MOTOR_POLES),
		  zAxis_(tmcSerial_, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_LIMIT_SWITCH_PIN, Z_STEPS_PER_MM),
		  pump_(PUMP_IN1, PUMP_IN2, PUMP_IN3, PUMP_IN4, PUMP_STEPS_PER_ML),
		  spinController_(spinMotor_, spinMotor_),
		  pumpController_(pump_),
		  zAxisController_(zAxis_),
		  orchestrator_(spinController_, pumpController_, zAxisController_),
		  telemetryService_(spinMotor_, spinMotor_, pump_, zAxis_, stateMachine_, orchestrator_),
		  displayService_(stateMachine_, displayHAL_),
		  webSocket_(recipeManager_)
	{}

	void Application::displayTaskEntry(void* param) {
		auto* app = static_cast<Application*>(param);
		for (;;) {
			if (displayMutex != nullptr && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
				app->refreshDisplay();
				xSemaphoreGive(displayMutex);
			}
			vTaskDelay(pdMS_TO_TICKS(200));
		}
	}

	void Application::refreshDisplay() {
		displayService_.update(telemetryService_.getSnapshot());
	}

	void Application::setup() {
		esp_task_wdt_deinit();

		Serial.begin(115200);
		delay(1000);
		Serial.println("OpenSilicon V1 - Application::setup()");

		// Bring the display up first so the board always has visual feedback,
		// independent of filesystem / homing / network init succeeding.
		displayHAL_.begin();
		displayHAL_.setBrightness(80);

		// Filesystem is only needed for recipe persistence. A mount failure must
		// not brick the device, so it is non-fatal.
		if (!LittleFS.begin(true)) {
			Serial.println("LittleFS mount failed - continuing without persistent recipes");
		} else {
			if (!LittleFS.exists("/recipes")) {
				LittleFS.mkdir("/recipes");
			}
			recipeManager_.begin();
		}

		zAxis_.begin();
		Serial.println("Z-axis homed.");

		pump_.begin();
		Serial.println("Pump initialized.");

		webSocket_.begin();

		spinMotor_.begin();
		Serial.println("Spin motor initialized.");

		setupEventRouting();

		displayMutex = xSemaphoreCreateMutex();

		// Render once on this thread before the display task exists, so the task
		// cannot preempt a half-finished SPI transaction.
		refreshDisplay();

		xTaskCreatePinnedToCore(displayTaskEntry, "Display", 4096, this, 2, nullptr, 1);

		Serial.println("Setup completed.");
	}

	void Application::loop() {
		stateMachine_.update();

		// Service controllers (pumps the HAL motion + emits completion events).
		zAxisController_.update();
		pumpController_.update();
		spinController_.update();

		// Advance the recipe and refresh telemetry.
		orchestrator_.update();
		telemetryService_.update();

		webSocket_.broadcastTelemetry(telemetryService_.getSnapshot());
		webSocket_.poll();

		delay(1);
	}

	void Application::setupEventRouting() {
		webSocket_.setEventPoster([this](const Domain::Event& ev) {
			stateMachine_.postEvent(ev);
		});

		webSocket_.setTelemetryProvider([this]() {
			return telemetryService_.getSnapshot();
		});

		webSocket_.setMotorControlCallback([this](const char* cmd, float param) {
			if (strcmp(cmd, "move_z") == 0) {
				zAxisController_.moveTo(static_cast<int32_t>(param * Z_STEPS_PER_MM));
			} else if (strcmp(cmd, "dispense") == 0) {
				pumpController_.dispense(param);
			} else if (strcmp(cmd, "aspirate") == 0) {
				pumpController_.aspirate(param);
			}
		});

		// Controller feedback drives the orchestrator forward.
		auto toOrchestrator = [this](const Domain::Event& ev) {
			orchestrator_.onControllerEvent(ev);
		};
		spinController_.setEventCallback(toOrchestrator);
		pumpController_.setEventCallback(toOrchestrator);
		zAxisController_.setEventCallback(toOrchestrator);

		// Orchestrator reports process progress back to the state machine.
		orchestrator_.setEventPoster([this](const Domain::Event& ev) {
			stateMachine_.postEvent(ev);
		});

		stateMachine_.setStateChangeCallback([this](Domain::MachineState newState) {
			switch (newState) {
				case Domain::MachineState::Running:
					if (orchestrator_.isActive()) {
						orchestrator_.resume();
					} else {
						orchestrator_.loadRecipe(recipeManager_.getCurrentRecipe());
						orchestrator_.start();
					}
					break;
				case Domain::MachineState::Paused:
					orchestrator_.pause();
					break;
				case Domain::MachineState::Idle:
				case Domain::MachineState::Fault:
					orchestrator_.stop();
					break;
				default:
					break;
			}
		});
	}
}
