#include "network/WebSocketServer.h"
#include "SimpleRecipeManager.h"
#include <LittleFS.h>

namespace Network {

	WebSocketServer::WebSocketServer(SimpleRecipeManager& recipeMgr)
	: server_(80), ws_("/ws"), recipeMgr_(recipeMgr) {}

	void WebSocketServer::begin() {
		WiFi.mode(WIFI_AP);
		WiFi.softAP("OpenSiliconSC", "coater123");
		Serial.println("WiFi AP started");
		Serial.print("IP: ");
		Serial.println(WiFi.softAPIP());

		server_.serveStatic("/", LittleFS, "/")
			.setDefaultFile("WebUI.html")
			.setTryGzipFirst(false);

		server_.addHandler(&ws_);

		ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
			onWsEvent(server, client, type, arg, data, len);
		});

		server_.begin();
		Serial.println("HTTP & WebSocket server started");
	}

	void WebSocketServer::onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
		switch(type) {
			case WS_EVT_CONNECT:
				Serial.printf("Client [%u] connected\n", client->id());
				break;
			case WS_EVT_DISCONNECT:
				Serial.printf("Client [%u] disconnected\n", client->id());
				break;
			case WS_EVT_DATA: {
				AwsFrameInfo *info = (AwsFrameInfo*)arg;
				if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
					char buffer[2048];
					if (len >= sizeof(buffer)) {
						sendError(client, "Message too long");
						return;
					}
					memcpy(buffer, data, len);
					buffer[len] = '\0';
					DynamicJsonDocument doc(3072);
					DeserializationError err = deserializeJson(doc, buffer);
					if (err) {
						sendError(client, "Invalid JSON");
						return;
					}
					handleCommand(client, doc);
				}
				break;
			}
			case WS_EVT_PONG:
			case WS_EVT_ERROR:
				break;
		}
	}

	void WebSocketServer::fillTelemetryData(JsonObject data, const Domain::RuntimeTelemetry& t) const {
		switch (t.currentState) {
			case Domain::MachineState::Idle: data["state"] = "IDLE"; break;
			case Domain::MachineState::Running: data["state"] = "RUNNING"; break;
			case Domain::MachineState::Paused: data["state"] = "PAUSED"; break;
			case Domain::MachineState::Fault: data["state"] = "FAULT"; break;
			default: data["state"] = "UNKNOWN";
		}
		data["targetSpinSpeed_rpm"] = t.targetSpinSpeed_rpm;
		data["actualSpinSpeed_rpm"] = t.actualSpinSpeed_rpm;
		data["dispensedVolume_ml"] = t.dispensedVolume_ml;
		data["pumpActive"] = t.pumpActive;
		data["zPosition_mm"] = t.zPosition_mm;
		data["zAxisMoving"] = t.zAxisMoving;
		data["currentStepIndex"] = t.currentStepIndex;
		data["totalSteps"] = t.totalSteps;
		data["stepProgress_percent"] = t.stepProgress_percent;
		data["faultCode"] = t.faultCode;
		data["faultDescription"] = t.faultDescription;
		data["timestamp_ms"] = t.lastUpdate_ms;
	}

	void WebSocketServer::serializeStep(JsonObject obj, const Domain::RecipeStep& step) {
		obj["stepIndex"] = step.stepIndex;
		obj["primeVolume_ml"] = step.primeVolume_ml;
		obj["dispenseVolume_ml"] = step.dispenseVolume_ml;
		obj["dispenseHeight_mm"] = step.dispenseHeight_mm;
		obj["spinHeight_mm"] = step.spinHeight_mm;
		obj["spinSpeed_rpm"] = step.spinSpeed_rpm;
		obj["rampTime_s"] = step.rampTime_s;
		obj["holdTime_s"] = step.holdTime_s;
	}

	void WebSocketServer::serializeRecipe(JsonObject obj, const Domain::Recipe& recipe) {
		obj["name"] = recipe.name;
		JsonArray steps = obj.createNestedArray("steps");
		for (const auto& step : recipe.steps) {
			serializeStep(steps.createNestedObject(), step);
		}
	}

	void WebSocketServer::sendTelemetry(AsyncWebSocketClient *client, const Domain::RuntimeTelemetry& t) {
		DynamicJsonDocument doc(512);
		doc["type"] = "telemetry";
		JsonObject data = doc.createNestedObject("data");
		fillTelemetryData(data, t);
		sendJSON(client, doc);
	}

	void WebSocketServer::sendAck(AsyncWebSocketClient *client) {
		DynamicJsonDocument doc(128);
		doc["status"] = "ok";
		doc["type"] = "ack";
		sendJSON(client, doc);
	}

	void WebSocketServer::handleCommand(AsyncWebSocketClient *client, const JsonDocument& doc) {
		const char* cmd = doc["cmd"];
		if (!cmd) {
			sendError(client, "Missing 'cmd'");
			return;
		}

		if (strcmp(cmd, "start") == 0) {
			if (recipeMgr_.getCurrentRecipe().steps.empty()) {
				sendError(client, "No recipe loaded");
				return;
			}
			if (eventPoster_) eventPoster_({Domain::EventType::StartRequested, 0});
			sendAck(client);
		} else if (strcmp(cmd, "pause") == 0) {
			if (eventPoster_) eventPoster_({Domain::EventType::PauseRequested, 0});
			sendAck(client);
		} else if (strcmp(cmd, "resume") == 0) {
			if (eventPoster_) eventPoster_({Domain::EventType::ResumeRequested, 0});
			sendAck(client);
		} else if (strcmp(cmd, "stop") == 0) {
			if (eventPoster_) eventPoster_({Domain::EventType::StopRequested, 0});
			sendAck(client);
		} else if (strcmp(cmd, "reset") == 0) {
			if (eventPoster_) eventPoster_({Domain::EventType::ResetRequested, 0});
			sendAck(client);
		} else if (strcmp(cmd, "move_z") == 0 || strcmp(cmd, "dispense") == 0 || strcmp(cmd, "aspirate") == 0) {
			if (motorControlCallback_) motorControlCallback_(cmd, doc["param"] | 0.0f);
			sendAck(client);
		} else if (strcmp(cmd, "get_recipes_list") == 0) {
			DynamicJsonDocument resp(512);
			resp["type"] = "recipes_list";
			JsonArray names = resp.createNestedArray("names");
			for (const auto& name : recipeMgr_.getRecipeNames()) {
				names.add(name);
			}
			sendJSON(client, resp);
		} else if (strcmp(cmd, "load_recipe") == 0) {
			const char* name = doc["name"];
			if (!name) { sendError(client, "Missing 'name'"); return; }
			if (recipeMgr_.loadRecipe(name)) {
				DynamicJsonDocument resp(1024);
				JsonObject root = resp.to<JsonObject>();
				root["type"] = "recipe";
				serializeRecipe(root, recipeMgr_.getCurrentRecipe());
				sendJSON(client, resp);
			} else {
				sendError(client, "Recipe not found");
			}
		} else if (strcmp(cmd, "set_recipe") == 0) {
			JsonObjectConst recipeObj = doc["recipe"].as<JsonObjectConst>();
			if (!recipeObj) { sendError(client, "Missing 'recipe'"); return; }
			Domain::Recipe recipe;
			strncpy(recipe.name, recipeObj["name"] | "", sizeof(recipe.name)-1);
			JsonArrayConst steps = recipeObj["steps"];
			for (JsonObjectConst s : steps) {
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
			recipeMgr_.setCurrentRecipe(recipe);
			Serial.printf("Recipe loaded: '%s' (%u steps)\n",
				recipe.name, static_cast<unsigned>(recipe.steps.size()));
			DynamicJsonDocument resp(1024);
			JsonObject root = resp.to<JsonObject>();
			root["type"] = "recipe";
			root["status"] = "ok";
			serializeRecipe(root, recipe);
			sendJSON(client, resp);
		} else if (strcmp(cmd, "save_recipe") == 0) {
			recipeMgr_.saveCurrentRecipe();
			sendAck(client);
		} else if (strcmp(cmd, "delete_recipe") == 0) {
			const char* name = doc["name"];
			if (!name) { sendError(client, "Missing 'name'"); return; }
			recipeMgr_.deleteRecipe(name);
			sendAck(client);
		} else if (strcmp(cmd, "get_telemetry") == 0) {
			if (!telemetryProvider_) {
				sendError(client, "Telemetry unavailable");
				return;
			}
			sendTelemetry(client, telemetryProvider_());
		} else {
			sendError(client, "Unknown command");
		}
	}

	void WebSocketServer::broadcastTelemetry(const Domain::RuntimeTelemetry& t) {
		const uint32_t now = millis();
		if (now - lastTelemetryBroadcastMs_ < TELEMETRY_BROADCAST_INTERVAL_MS) {
			return;
		}
		lastTelemetryBroadcastMs_ = now;

		if (ws_.count() == 0) {
			return;
		}
		if (!ws_.availableForWriteAll()) {
			return;
		}

		DynamicJsonDocument doc(512);
		doc["type"] = "telemetry";
		JsonObject data = doc.createNestedObject("data");
		fillTelemetryData(data, t);

		String output;
		serializeJson(doc, output);
		ws_.textAll(output);
	}

	void WebSocketServer::sendError(AsyncWebSocketClient *client, const char* message) {
		DynamicJsonDocument doc(128);
		doc["type"] = "error";
		doc["message"] = message;
		sendJSON(client, doc);
	}

	void WebSocketServer::sendJSON(AsyncWebSocketClient *client, const JsonDocument& doc) {
		String output;
		serializeJson(doc, output);
		client->text(output);
	}

	void WebSocketServer::poll() {
		static uint32_t lastCleanup = 0;
		if (millis() - lastCleanup > 250) {
			ws_.cleanupClients();
			lastCleanup = millis();
		}
	}
}
