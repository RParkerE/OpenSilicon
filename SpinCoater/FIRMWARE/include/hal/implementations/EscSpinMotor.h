#pragma once
#include "hal/interfaces/ISpinMotor.h"
#include "hal/interfaces/IRpmSensor.h"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <Arduino.h>
#include <ESP32_DSHOT.h>

namespace HAL {
    /**
     * Telemetry decoded from the ESC over the single signal wire via
     * bidirectional DShot (GCR-encoded reply). eRPM is always available;
     * temperature/voltage/current are only populated if Extended DShot
     * Telemetry (EDT) is enabled on the ESC.
     */
    struct DShotTelemetry {
        bool valid = false;
        int32_t eRPM = 0;
        float temperature_C = 0;
        float voltage_V = 0;
        float current_A = 0;
        uint32_t crcError = 0;
    };

    /**
     * AM32 ESC driven over bidirectional DShot300.
     *
     * There is no separate UART telemetry wire: RPM feedback comes back on the
     * SAME signal pin as a GCR-encoded reply ~30us after each command. That
     * TX->RX flip is too fast for the legacy RMT driver, so the timing-critical
     * work is handled by the ESP32_DSHOT library (low-level RMT).
     *
     * Bidirectional DShot is auto-negotiated by signal polarity: AM32 detects
     * the inverted (idle-high) frames this driver sends and starts replying
     * with eRPM. There is no persistent "enable bidir" setting in AM32 -- if
     * telemetry never decodes, the ESC firmware is usually too old (update it).
     *
     * Hardware requirements:
     *   - External ~1-2k pull-up from the signal pin to 3.3V.
     *   - Recent AM32 firmware (older builds have flaky bidir telemetry).
     */
    class EscSpinMotor : public ISpinMotor, public IRpmSensor {
        public:
            EscSpinMotor(gpio_num_t pin, uint32_t dshotSpeedKbps, uint8_t motorPoles = 14);
            void begin();
            void update();

            void setTargetSpeed(int32_t rpm) override;
            void stop() override;
            int32_t getTargetSpeed() const override;

            int32_t readRPM() override;
            bool isConnected() const override;

            const DShotTelemetry& getTelemetry() const { return lastTelemetry_; }

        private:
            enum class EscCommandType : uint8_t { SetTarget, Stop };

            struct EscCommand {
                EscCommandType type;
                int32_t rpm;
            };

            gpio_num_t pin_;
            uint8_t motorPoles_;
            int32_t targetRPM_ = 0;
            std::atomic<int32_t> reportedTargetRPM_{0};

            DSHOT dshot_;

            QueueHandle_t cmdQueue_ = nullptr;
            DShotTelemetry lastTelemetry_;
            unsigned long lastTelemetryTime_ = 0;
            uint32_t lastOkCount_ = 0;
            std::atomic<int32_t> actualRPM_{0};

            TaskHandle_t taskHandle_ = nullptr;

            // Feedforward + PI speed controller state (ESC task thread only).
            float integrator_ = 0.0f;

            void processCommands();
            void readTelemetry();
            int32_t computeThrottle(int32_t target, int32_t actual);
            void writeThrottle(int32_t throttle);
            void updateLoop();
            static void taskEntry(void* param);
            bool enqueueCommand(EscCommandType type, int32_t rpm);
    };
}
