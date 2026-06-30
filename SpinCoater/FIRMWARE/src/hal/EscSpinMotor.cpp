#include "hal/implementations/EscSpinMotor.h"
#include "common/configs.h"
#include "common/pin_mapping.h"
#include <Arduino.h>

namespace HAL {
    static const int32_t MIN_OPERATING_RPM = 200;

    // Throttle units for DSHOT::set(int): 0..2000. The library adds the +48
    // DShot offset internally, so 0 here = lowest spinning throttle (DShot 48).
    static const int32_t THROTTLE_MAX = 1999;
    static const int32_t THROTTLE_MIN_SPIN = 100;   // floor once the motor is turning
    static const int32_t STARTUP_THROTTLE = 200;    // open-loop kick to break stiction

    // Closed-loop speed control. Under the near-constant load of a spin coater
    // the ESC maps throttle (0..THROTTLE_MAX) onto speed roughly linearly, so a
    // linear feedforward seeds the loop and a PI controller trims the nonlinear
    // remainder and holds the setpoint. Tune these against your motor/load.
    static const float SPEED_KP = 0.10f;              // throttle units per RPM error
    static const float SPEED_KI = 0.30f;              // throttle units per RPM error per second
    static const float THROTTLE_FF_PER_RPM = 0.12f;   // feedforward seed (throttle per target RPM)

    EscSpinMotor::EscSpinMotor(gpio_num_t pin, uint32_t dshotSpeedKbps, uint8_t motorPoles)
        : pin_(pin),
          motorPoles_(motorPoles)
    {
        (void)dshotSpeedKbps;  // speed/protocol is fixed at DSHOT300_BIDIR (see begin())
    }

    bool EscSpinMotor::enqueueCommand(EscCommandType type, int32_t rpm) {
        if (cmdQueue_ == nullptr) {
            return false;
        }
        EscCommand cmd{type, rpm};
        return xQueueSend(cmdQueue_, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
    }

    void EscSpinMotor::begin() {
        pinMode(STATUS_LED_PIN, OUTPUT);
        digitalWrite(STATUS_LED_PIN, LOW);

        // Bidirectional DShot300 on a single wire: throttle out, GCR eRPM in.
        // DShot1200 bidir is unreliable on ESP32; 300 is the sweet spot.
        if (!dshot_.begin(static_cast<int>(pin_), DSHOT::DSHOT300_BIDIR)) {
            Serial.println("ERROR: DShot begin failed (no free RMT channels?)");
        }

        // Spin-direction commands must be repeated several times to take effect.
        for (int i = 0; i < 10; i++) {
            dshot_.cmd(DSHOT::DSHOT_CMD_SPIN_DIRECTION_NORMAL);
            delay(1);
        }

        DSHOT::arm();  // ~1.5s: line low, motor-stop, zero throttle
        Serial.println("ESC armed");

        cmdQueue_ = xQueueCreate(8, sizeof(EscCommand));
        if (cmdQueue_ == nullptr) {
            Serial.println("ERROR: ESC command queue creation failed");
            return;
        }

        BaseType_t created = xTaskCreatePinnedToCore(
            taskEntry,
            "EscSpin",
            4096,
            this,
            4,
            &taskHandle_,
            0
        );
        if (created != pdPASS) {
            Serial.println("ERROR: EscSpin task creation failed");
        }
    }

    void EscSpinMotor::taskEntry(void* param) {
        auto* self = static_cast<EscSpinMotor*>(param);
        TickType_t lastWake = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(ESC_UPDATE_INTERVAL_MS);

        for (;;) {
            self->updateLoop();
            vTaskDelayUntil(&lastWake, period);
        }
    }

    void EscSpinMotor::update() {}

    void EscSpinMotor::processCommands() {
        EscCommand cmd;
        while (xQueueReceive(cmdQueue_, &cmd, 0) == pdTRUE) {
            if (cmd.type == EscCommandType::SetTarget) {
                int32_t rpm = cmd.rpm;
                if (rpm > MAX_RPM) rpm = MAX_RPM;
                if (rpm < 0) rpm = 0;
                targetRPM_ = rpm;
                reportedTargetRPM_.store(rpm, std::memory_order_relaxed);
            } else {
                targetRPM_ = 0;
                reportedTargetRPM_.store(0, std::memory_order_relaxed);
                integrator_ = 0.0f;
            }
        }
    }

    void EscSpinMotor::readTelemetry() {
        // erpm_us is the eRPM period in microseconds (0 = stopped / unavailable).
        // Updated by the library's RX ISR after each bidirectional frame.
        const uint16_t period_us = dshot_.erpm_us;
        const uint32_t polePairs = motorPoles_ ? (motorPoles_ / 2) : 1;

        int32_t mechRPM = 0;
        int32_t eRPM = 0;
        if (period_us > 0) {
            eRPM = static_cast<int32_t>(60000000UL / period_us);  // electrical RPM
            mechRPM = eRPM / static_cast<int32_t>(polePairs);
        }

        // Light EMA (alpha 0.4) smooths the jittery per-frame eRPM for both the
        // control loop and telemetry; snap straight to 0 when the motor stops.
        if (mechRPM == 0) {
            actualRPM_.store(0, std::memory_order_relaxed);
        } else {
            const int32_t prev = actualRPM_.load(std::memory_order_relaxed);
            actualRPM_.store(prev + ((mechRPM - prev) * 2) / 5, std::memory_order_relaxed);
        }

        if (dshot_.telem_ok_cnt != lastOkCount_) {
            lastOkCount_ = dshot_.telem_ok_cnt;
            lastTelemetryTime_ = micros();
            lastTelemetry_.valid = true;
            lastTelemetry_.eRPM = eRPM;
            // Only meaningful if Extended DShot Telemetry is enabled on the ESC.
            lastTelemetry_.temperature_C = dshot_.telem[DSHOT::TELEM_TEMPERATURE];
            lastTelemetry_.voltage_V = dshot_.telem[DSHOT::TELEM_VOLTAGE] * 0.25f;
            lastTelemetry_.current_A = dshot_.telem[DSHOT::TELEM_CURRENT];
        }
        lastTelemetry_.crcError = dshot_.telem_cnt - dshot_.telem_ok_cnt;
    }

    void EscSpinMotor::writeThrottle(int32_t throttle) {
        if (throttle <= 0) {
            dshot_.cmd(DSHOT::DSHOT_CMD_MOTOR_STOP);
            return;
        }
        if (throttle > THROTTLE_MAX) throttle = THROTTLE_MAX;
        dshot_.set(static_cast<int>(throttle));
    }

    // Feedforward + PI with conditional-integration anti-windup. Returns the
    // throttle (THROTTLE_MIN_SPIN..THROTTLE_MAX) to hold `target` given the
    // measured `actual`. Caller guarantees target > MIN_OPERATING_RPM.
    int32_t EscSpinMotor::computeThrottle(int32_t target, int32_t actual) {
        const float dt = ESC_UPDATE_INTERVAL_MS * 0.001f;
        const float error = static_cast<float>(target - actual);
        const float ff = THROTTLE_FF_PER_RPM * static_cast<float>(target);

        float output = ff + SPEED_KP * error + integrator_;

        // Only integrate when not saturated, so the integrator can't wind up.
        if (output > THROTTLE_MIN_SPIN && output < THROTTLE_MAX) {
            integrator_ += SPEED_KI * error * dt;
            if (integrator_ < 0.0f) integrator_ = 0.0f;
            if (integrator_ > THROTTLE_MAX) integrator_ = THROTTLE_MAX;
            output = ff + SPEED_KP * error + integrator_;
        }

        // Open-loop kick until the motor is actually turning (break stiction).
        if (actual < MIN_OPERATING_RPM && output < STARTUP_THROTTLE) {
            output = STARTUP_THROTTLE;
        }

        if (output > THROTTLE_MAX) output = THROTTLE_MAX;
        if (output < THROTTLE_MIN_SPIN) output = THROTTLE_MIN_SPIN;
        return static_cast<int32_t>(output);
    }

    void EscSpinMotor::updateLoop() {
        static uint32_t debugCounter = 0;
        if (debugCounter++ % 100 == 0) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        }

        processCommands();
        readTelemetry();

        const int32_t localTarget = targetRPM_;
        const int32_t localActual = actualRPM_.load(std::memory_order_relaxed);

        int32_t throttle = 0;
        if (localTarget > MIN_OPERATING_RPM) {
            throttle = computeThrottle(localTarget, localActual);
        } else {
            integrator_ = 0.0f;
        }
        writeThrottle(throttle);
    }

    void EscSpinMotor::setTargetSpeed(int32_t rpm) {
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < 0) rpm = 0;
        if (!enqueueCommand(EscCommandType::SetTarget, rpm)) {
            Serial.println("ERROR: failed to queue ESC target RPM");
        }
    }

    void EscSpinMotor::stop() {
        if (!enqueueCommand(EscCommandType::Stop, 0)) {
            Serial.println("ERROR: failed to queue ESC stop");
        }
    }

    int32_t EscSpinMotor::getTargetSpeed() const {
        return reportedTargetRPM_.load(std::memory_order_relaxed);
    }

    int32_t EscSpinMotor::readRPM() {
        return actualRPM_.load(std::memory_order_relaxed);
    }

    bool EscSpinMotor::isConnected() const {
        return lastTelemetry_.valid && (micros() - lastTelemetryTime_ < 500000);
    }
}
