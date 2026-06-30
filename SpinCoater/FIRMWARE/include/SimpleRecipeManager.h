#pragma once
#include "domain/Recipe.h"
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <LittleFS.h>
#include <ArduinoJson.h>

class SimpleRecipeManager {
	public:
		SimpleRecipeManager() = default;

		// Load persisted recipes. Must be called AFTER LittleFS.begin(), never
		// from the constructor (globals are built before the filesystem mounts).
		void begin() {
			loadAllFromLittleFS();
		}

		std::vector<std::string> getRecipeNames() const {
			std::vector<std::string> names;
			for (const auto& pair : savedRecipes_) {
				names.push_back(pair.first);
			}
			return names;
		}

		Domain::Recipe getCurrentRecipe() const {
			return currentRecipe_;
		}

		bool loadRecipe(const std::string& name) {
			auto it = savedRecipes_.find(name);
			if (it != savedRecipes_.end()) {
				currentRecipe_ = it->second;
				return true;
			}
			return false;
		}

		void setCurrentRecipe(const Domain::Recipe& recipe) {
			currentRecipe_ = recipe;
		}

		void saveCurrentRecipe() {
			if (currentRecipe_.name[0] != '\0') {
				savedRecipes_[currentRecipe_.name] = currentRecipe_;
				saveToLittleFS(currentRecipe_.name, currentRecipe_);
			}
		}

		void deleteRecipe(const std::string& name) {
			savedRecipes_.erase(name);
			String filename = "/recipes/" + String(name.c_str()) + ".json";
			LittleFS.remove(filename);
		}

	private:
		std::map<std::string, Domain::Recipe> savedRecipes_;
		Domain::Recipe currentRecipe_;

		void loadAllFromLittleFS() {
			File dir = LittleFS.open("/recipes");
			if (!dir || !dir.isDirectory()) return;
			File entry = dir.openNextFile();
			while (entry) {
				String name = entry.name();
				if (name.endsWith(".json")) {
					String recipeName = name.substring(9, name.length()-5);
					Domain::Recipe recipe;
					if (parseRecipe(entry, recipe)) {
						savedRecipes_[recipeName.c_str()] = recipe;
					}
				}
				entry = dir.openNextFile();
			}
		}

		bool parseRecipe(File file, Domain::Recipe& recipe) {
			DynamicJsonDocument doc(2048);
			DeserializationError err = deserializeJson(doc, file);
			if (err) return false;
			strncpy(recipe.name, doc["name"] | "", sizeof(recipe.name)-1);
			JsonArray steps = doc["steps"];
			for (JsonObject s : steps) {
				Domain::RecipeStep step;
				step.stepIndex = s["stepIndex"] | 0;
				step.primeVolume_ml = s["primeVolume_ml"] | 0.0f;
				step.dispenseVolume_ml = s["dispenseVolume_ml"] | 0.0f;
				step.dispenseHeight_mm = s["dispenseHeight_mm"] | 0.0f;
				step.spinHeight_mm = s["spinHeight_mm"] | 0.0f;
				step.spinSpeed_rpm = s["spinSpeed_rpm"] | 0.0f;
				step.rampTime_s = s["rampTime_s"] | 0.0f;
				step.holdTime_s = s["holdTime_s"] | 0.0f;
				recipe.steps.push_back(step);
			}
			return true;
		}

		void saveToLittleFS(const std::string& name, const Domain::Recipe& recipe) {
			String filename = "/recipes/" + String(name.c_str()) + ".json";
			File file = LittleFS.open(filename, FILE_WRITE);
			if (!file) return;
			DynamicJsonDocument doc(2048);
			doc["name"] = recipe.name;
			JsonArray steps = doc.createNestedArray("steps");
			for (const auto& s : recipe.steps) {
				JsonObject step = steps.createNestedObject();
				step["stepIndex"] = s.stepIndex;
				step["primeVolume_ml"] = s.primeVolume_ml;
				step["dispenseVolume_ml"] = s.dispenseVolume_ml;
				step["dispenseHeight_mm"] = s.dispenseHeight_mm;
				step["spinHeight_mm"] = s.spinHeight_mm;
				step["spinSpeed_rpm"] = s.spinSpeed_rpm;
				step["rampTime_s"] = s.rampTime_s;
				step["holdTime_s"] = s.holdTime_s;
			}
			serializeJson(doc, file);
			file.close();
		}
};
