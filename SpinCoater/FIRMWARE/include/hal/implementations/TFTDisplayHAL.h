#pragma once
#include "hal/interfaces/IDisplay.h"
#include <TFT_eSPI.h>

namespace HAL {

	/**
	 * TFT_eSPI-based implementation of IDisplay.
	 * Receives a DisplayModel and draws the corresponding page.
	 */
	class TFTDisplayHAL : public IDisplay {
		public:
			TFTDisplayHAL(TFT_eSPI& tft) : tft_(tft) {}

			void begin() override;
			void render(const Domain::DisplayModel& model) override;
			void clear() override;
			void setBrightness(uint8_t percent) override;

		private:
			TFT_eSPI& tft_;
	};
}
