#pragma once
#include <vector>
#include <cstdint>

namespace Domain {

	/**
	 * A single step of a spin-coating recipe.
	 *
	 * Execution order per step:
	 *   1. (optional) aspirate `primeVolume_ml` to prime the syringe
	 *   2. lower the head to `dispenseHeight_mm`
	 *   3. dispense `dispenseVolume_ml` of resist onto the wafer
	 *   4. raise the head to `spinHeight_mm` to clear the spinning wafer
	 *   5. ramp the spindle to `spinSpeed_rpm` over `rampTime_s`
	 *   6. hold at speed for `holdTime_s`, then ramp back down
	 */
	struct RecipeStep {
		float primeVolume_ml = 0.0f;     // Aspirate before dispensing (0 = skip)
		float dispenseVolume_ml = 0.0f;  // Volume to dispense (0 = no dispense)
		float dispenseHeight_mm = 0.0f;  // Head height while dispensing
		float spinHeight_mm = 0.0f;      // Head height while spinning
		float spinSpeed_rpm = 0.0f;      // Target spindle speed
		float rampTime_s = 0.0f;         // Time to reach (and to leave) target speed
		float holdTime_s = 0.0f;         // Time to hold at target speed
		uint8_t stepIndex = 0;           // 1-based index for the UI
	};

	/**
	 * Complete recipe consisting of ordered steps.
	 */
	struct Recipe {
		char name[32] = {0};
		std::vector<RecipeStep> steps;
	};
}
