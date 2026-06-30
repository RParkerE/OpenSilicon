#pragma once
#include "domain/DisplayPage.h"
#include "domain/DisplayModel.h"
#include "domain/RuntimeTelemetry.h"
#include <cstdint>

namespace HAL {

	/**
	 * Hardware abstraction for the TFT display.
	 * The DisplayService fills a DisplayModel, this interface renders it.
	 */
	class IDisplay {
		public:
			virtual ~IDisplay() = default;

			// Initialize display hardware
			virtual void begin() = 0;

			// Render a complete page from the provided model.
			// The implementation decides which widgets to update.
			virtual void render(const Domain::DisplayModel& model) = 0;

			// Clear screen (fault reset, etc.)
			virtual void clear() = 0;

			// Set backlight brightness 0 - 100%
			virtual void setBrightness(uint8_t percent) = 0;
	};
}
