#pragma once
#include <cstdint>

namespace HAL {

	/**
	 * Abstract RPM feedback sensor.
	 * V1 uses ESCRpmSensor (no Hall Effect hardware)
	 * V2 will add a check using HallEffectRpmSensor.
	 */
	class IRpmSensor {
		public:
			virtual ~IRpmSensor() = default;

			// Read the current rotational speed in RPM
			// Returns 0 if no valid reading
			virtual int32_t readRPM() = 0;

			// Returns true if the sensor provides a valid signal
			virtual bool isConnected() const = 0;
	};
}
