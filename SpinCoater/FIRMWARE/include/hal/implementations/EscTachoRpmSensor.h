#pragma once
#include "hal/interfaces/IRpmSensor.h"
#includ <cstdint>

namespace HAL {

	/**
	 * Reads RPM from the ESC's tachometer PWM output
	 * Assusmes a digital pin receives a pulse train where:
	 * 	frequency ~ motor RPM
	 * 	typically one pulse per rev
	 *
	 * Uses a hardware interrupt to measure the periof between successive rising edges
	 * The conversion factor is set at construction
	 */
	class EscTachoRpmSensor : public IRpmSensor {
		public:
			EscTachoRpmSensor(uint8_t pin, uint8_t pulsesPerRev = 1);
			void begin();
			int32_t readRPM() override;
			bool isConnected() const override;

		private:
			const uint8_t pin_;
			const uint8_t pulsesPerRev_;

			volatile uint32_t lastPulseTime_us = 0;
			volatile uint32_t pulsePeriod_us = 0;

			static constexpr uint32_t SIGNAL_TIMEOUT_US = 500'000;

			static void IRAM_ATTR isrHandler(void* arg);

			static EscTachoRpmSensor* instance_;
	};
}
