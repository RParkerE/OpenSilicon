#pragma once
#include "domain/DisplayPage.h"
#include "domainRuntimeTelemetry.h"
#include "domain/Recipe.h"
#include <cstring>

namespace Domain {

	/**
	 * Pure data model representing what the display should show.
	 * Built by DisplayService from RuntimeTelemetry and other sources.
	 * The HAL display driver renders this model without any logic.
	 */
	struct DisplayModel {
		DisplayPage activePage = DisplayPage::Status;

		// Status page data
		char statusMessage[64] = "Idle";

		// Recipe page data
		char recipeName[32] = "";
		uint8_t recipeStepCount = 0;
		RecipeStep recipeSteps[10];

		// Active process page data
		int32_t targetSpeed = 0;
		int32_t actualSpeed = 0;
		float dispensedVolume = 0.0f;
		uint8_t currentStep = 0;
		uint8_t totalSteps = 0;
		float stepProgress = 0.0f;

		// Fault page data
		uint16_t faultCode = 0;
		char faultDescription[64] = "";
	};
}
