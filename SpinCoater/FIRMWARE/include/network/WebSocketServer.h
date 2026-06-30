#pragma once
#include "common/configs.h"
#include "domain/Event.h"
#include "domain/Recipe.h"
#include "domain/RuntimeTelemetry.h"
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <functional>
#include <vector>

class SimpleRecipeManager;

namespace Network {

	/**
	 * WebSocket interface.
	 * Receives user commands and translates them into events for the StateMachine.
	 * Also broadcasts telemetry snapshots to connected clients; it never accesses
	 * the controllers or hardware directly.
	 */
	class WebSocketServer {
		public:
			using EventPoster = std::function<void(const Domain::Event&)>;
			using MotorControlCallback = std::function<void(const char* cmd, float param)>;
			using TelemetryProvider = std::function<Domain::RuntimeTelemetry()>;
			WebSocketServer(SimpleRecipeManager& recipeMgr);
			void begin();

			void setEventPoster(EventPoster poster) { eventPoster_ = poster; }
			void setMotorControlCallback(MotorControlCallback cb) { motorControlCallback_ = cb; }
			void setTelemetryProvider(TelemetryProvider provider) { telemetryProvider_ = provider; }

			// Push telemetry to all connected clients
			void broadcastTelemetry(const Domain::RuntimeTelemetry& telemetry);

			void poll();

		private:
			AsyncWebServer server_;
			AsyncWebSocket ws_;
			EventPoster eventPoster_;
			MotorControlCallback motorControlCallback_;
			TelemetryProvider telemetryProvider_;
			SimpleRecipeManager& recipeMgr_;
			uint32_t lastTelemetryBroadcastMs_ = 0;

			void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
			void handleCommand(AsyncWebSocketClient *client, const JsonDocument& doc);
			void fillTelemetryData(JsonObject data, const Domain::RuntimeTelemetry& telemetry) const;
			static void serializeStep(JsonObject obj, const Domain::RecipeStep& step);
			static void serializeRecipe(JsonObject obj, const Domain::Recipe& recipe);
			void sendTelemetry(AsyncWebSocketClient *client, const Domain::RuntimeTelemetry& telemetry);
			void sendAck(AsyncWebSocketClient *client);
			void sendError(AsyncWebSocketClient *client, const char* message);
			void sendJSON(AsyncWebSocketClient *client, const JsonDocument& doc);
	};
}
