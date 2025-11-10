// ---------------------------------------------
//  IMPORTS
// ---------------------------------------------
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"

// Motor Drivers
#include <TMCStepper.h>
#include <AccelStepper.h>

// Display
#include <TFT_eSPI.h>

// Networking & WebSocket
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// PID Controller
#include <PID_RT.h>

#include "driver/rmt.h"
#include "HardwareSerial.h"
#include <elapsedMillis.h>

// ---------------------------------------------
//  PIN ASSIGNMENTS
// ---------------------------------------------
#define DRIVER_ADDRESS 0b00
#define PIN_TMC_STEP 25
#define PIN_TMC_DIR 26
#define PIN_TMC_EN 27
#define PIN_TMC_TX 17
#define PIN_TMC_RX 16
HardwareSerial TMCSerial(2);
#define PIN_LIMIT_SWITCH 19

#define PIN_ULN_IN1 32
#define PIN_ULN_IN2 21
#define PIN_ULN_IN3 14
#define PIN_ULN_IN4 33

#define PIN_ESC_DSHOT 13
#define PIN_ESC_RX 9
HardwareSerial TelSerial(1);

#define ILI9341_DRIVER
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS 15
#define TFT_DC 22
#define TFT_RST 4

#define STATUS_LED_PIN 2

// ---------------------------------------------
//  WiFi & WebSocket
// ---------------------------------------------
const char *WIFI_SSID = "YourSSID";
const char *WIFI_PASSWORD = "YourPassword";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------------------------------------
//  MOTION LIMITS & CONFIG
// ---------------------------------------------
#define DEFAULT_DISPENSE_SPEED 600
#define DEFAULT_COAT_SPEED 3000
#define MIN_COAT_SPEED 300
#define MAX_COAT_SPEED 8000
#define INCREMETS_COAT_SPEED 100
#define DEFAULT_COAT_TIME 30000
#define MIN_COAT_TIME 1000
#define MAX_COAT_TIME 80000
#define INCREMETS_COAT_TIME 1000
#define DEFAULT_RAMPUP_TIME 3000
#define MIN_RAMPUP_TIME 1000
#define MAX_RAMPUP_TIME 6000
#define DEFAULT_RAMPDOWN_TIME 1500
#define MIN_RAMPDOWN_TIME 500
#define MAX_RAMPDOWN_TIME 5000

// DSHOT 1200 Configuration
#define MOTOR_POLE_PAIRS 7
#define R_SENSE 0.11f
#define HALFSTEP 8
#define DSHOT_BIT_0_HIGH 6
#define DSHOT_BIT_0_LOW  19
#define DSHOT_BIT_1_HIGH 13
#define DSHOT_BIT_1_LOW  13
#define DSHOT_FRAME_SIZE 16
#define DSHOT_CMD_MOTOR_STOP            0
#define DSHOT_CMD_BEEP1                 1
#define DSHOT_CMD_BEEP2                 2
#define DSHOT_CMD_BEEP3                 3
#define DSHOT_CMD_BEEP4                 4
#define DSHOT_CMD_BEEP5                 5
#define DSHOT_CMD_ESC_INFO              6
#define DSHOT_CMD_SPIN_DIRECTION_1      7
#define DSHOT_CMD_SPIN_DIRECTION_2      8
#define DSHOT_CMD_3D_MODE_OFF           9
#define DSHOT_CMD_3D_MODE_ON            10
#define DSHOT_CMD_SETTINGS_REQUEST      11
#define DSHOT_CMD_SAVE_SETTINGS         12
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL 20
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED 21
rmt_item32_t dshotPacket[DSHOT_FRAME_SIZE];

#define TASK_STACK_DEFAULT 4096
#define TASK_PRIORITY_HIGH 4
#define TASK_PRIORITY_MED 3
#define TASK_PRIORITY_LOW 2

#define Z_HEIGHT_MAX 300
#define HOMING_SPEED 1000
#define HOMING_OFFSET 5.0

// ---------------------------------------------
//  STRUCTURES
// ---------------------------------------------
struct SpinSettings {
    float zHeight;
    float dispenseVol;
    int targetRPM;
    int runTime;
    int rampTime;
};

struct SpinTelemetry {
    float currentZ;
    int currentRPM;
    String runStage;
    bool wsConnected;
};

SpinSettings currentSettings;
SpinTelemetry currentTelemetry;

// TFT
TFT_eSPI tft = TFT_eSPI();

// NEMA17 - TMC2209
constexpr uint32_t steps_per_mm = 200;
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper = AccelStepper(stepper.DRIVER, PIN_TMC_STEP, PIN_TMC_DIR);

// 28BYJ-48 - ULN2003
constexpr uint32_t steps_per_ul = 410;
AccelStepper dispensor(AccelStepper::HALF4WIRE, PIN_ULN_IN1, PIN_ULN_IN2, PIN_ULN_IN3, PIN_ULN_IN4);

// ESC
elapsedMillis time_ramping = 0;
elapsedMillis time_coating = 0;
hw_timer_t *timer = NULL;
uint8_t receivedBytes = 0;
volatile bool requestTelemetry = false;
int16_t ESC_telemetry[5];
uint16_t disp_speed, coat_speed;
int32_t rampup_time, rampdown_time, coat_time;
uint32_t currentTime;
uint8_t temperature = 0;
uint8_t temperatureMax = 0;
float voltage = 0;
float voltageMin = 99;
uint32_t current = 0;
uint32_t currentMax = 0;
uint32_t erpm = 0;
uint32_t erpmMax = 0;
uint32_t rpm = 0;
uint32_t rpmMAX = 0;
uint32_t kv = 0;
uint32_t kvMax = 0;
uint16_t dshotUserInputValue = 0;
uint16_t dshotmin = 48;
uint16_t dshotmax = 2047;
double motor_setRPM = 0;
double motor_actualRPM = 0;
double pid_output = 0;
double pid_out_min = 48;
double pid_out_max = 2047;
double Kp = 0.015, Ki = 0.8, Kd = 0.0;
const int sample_time = 1;

PID_RT motorPID(Kp, Ki, Kd, sample_time);

enum state {
    STATE_IDLE,
    STATE_HOMING,
    STATE_POSITIONING,
    STATE_DISPENSING,
    STATE_RAMPUP,
    STATE_RAMPDOWN,
    STATE_SPINNING
};
state current_state, previous_state = STATE_IDLE;

bool isHomed = false;

SemaphoreHandle_t xTelemetrySemaphore = NULL;
SemaphoreHandle_t xMotionSemaphore = NULL;
SemaphoreHandle_t xStateSemaphore = NULL;
SemaphoreHandle_t xDisplaySemaphore = NULL;

// ---------------------------------------------
//  FUNCTION DECLARATIONS
// ---------------------------------------------
void handleWebRequest();
void updateSettings(float zHeight, float dispenseVol, int targetRPM, int runTime, int rampTime);
void moveZAxis(float targetHeight);
void dispenseResist(float volumeUL);
void runSpinCycle();
void sendStatusUpdate();
bool validateInputs(float zHeight, float dispenseVol, int targetRPM, int runTime, int rampTime);
uint8_t mapStageToByte(String stage);
uint8_t crc8(uint8_t *data, uint8_t len);
void initDshot();
void dshotOutput(uint16_t value, bool telemetry);
void homeZAxis();
void updateDisplay();

// ---------------------------------------------
//  DSHOT FUNCTIONS - DSHOT 1200
// ---------------------------------------------
uint8_t dshotCRC(uint16_t frame) {
    uint8_t crc = 0;
    uint16_t crc_data = frame;
    
    for (int i = 0; i < 3; i++) {
        crc ^= (crc_data & 0x01);
        crc_data >>= 1;
        crc ^= (crc_data & 0x01);
        crc_data >>= 1;
        crc ^= (crc_data & 0x01);
        crc_data >>= 1;
        crc ^= (crc_data & 0x01);
        crc_data >>= 1;
    }
    return crc;
}

void initDshot() {
    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)PIN_ESC_DSHOT, RMT_CHANNEL_0);
    rmt_tx_config.clk_div = 8;
    rmt_tx_config.mem_block_num = 1;
    rmt_tx_config.tx_config.loop_en = false;
    rmt_tx_config.tx_config.carrier_en = false;
    rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_tx_config.tx_config.idle_output_en = true;
    
    ESP_ERROR_CHECK(rmt_config(&rmt_tx_config));
    ESP_ERROR_CHECK(rmt_driver_install(RMT_CHANNEL_0, 0, 0));
}

void dshotOutput(uint16_t throttle, bool telemetry) {
    if (throttle > 2047) throttle = 2047;
    
    uint16_t frame = (throttle << 1) | (telemetry ? 1 : 0);
    uint8_t crc = dshotCRC(frame);
    frame = (frame << 4) | crc;
    
    for (int i = 0; i < DSHOT_FRAME_SIZE; i++) {
        bool bit_val = (frame >> (15 - i)) & 0x01;
        
        if (bit_val) {
            dshotPacket[i].level0 = 1;
            dshotPacket[i].duration0 = DSHOT_BIT_1_HIGH;
            dshotPacket[i].level1 = 0;
            dshotPacket[i].duration1 = DSHOT_BIT_1_LOW;
        } else {
            dshotPacket[i].level0 = 1;
            dshotPacket[i].duration0 = DSHOT_BIT_0_HIGH;
            dshotPacket[i].level1 = 0;
            dshotPacket[i].duration1 = DSHOT_BIT_0_LOW;
        }
    }
    
    rmt_write_items(RMT_CHANNEL_0, dshotPacket, DSHOT_FRAME_SIZE, false);
}

void dshotCommand(uint16_t command) {
    for (int i = 0; i < 10; i++) {  
        dshotOutput(command, false);
        delayMicroseconds(250);
    }
}

void dshotStop() {
    dshotOutput(0, false);
}

void initESC() {    
    dshotCommand(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
    delay(100);
    
    for (int i = 0; i < 400; i++) {  
        dshotOutput(0, false);
        delay(10);
    }
    
    for (int throttle = 0; throttle <= 200; throttle += 10) {
        dshotOutput(throttle, false);
        delay(30);
    }
    
    delay(200);
    
    for (int throttle = 200; throttle >= 0; throttle -= 10) {
        dshotOutput(throttle, false);
        delay(30);
    }
    
    for (int i = 0; i < 50; i++) {
        dshotOutput(0, false);
        delay(20);
    }
}

// ---------------------------------------------
// DIAGNOSTICS
// ---------------------------------------------
void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(delayMs);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(delayMs);
    }
}

// ---------------------------------------------
//  HOMING
// ---------------------------------------------
void homeZAxis() {
    isHomed = false;

    if (digitalRead(PIN_LIMIT_SWITCH) == LOW) {
        stepper.setMaxSpeed(1000);
        stepper.setAcceleration(500);
        stepper.move(steps_per_mm * 10);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        delay(100); 
    }
    
    stepper.setMaxSpeed(HOMING_SPEED);
    stepper.setAcceleration(500);
    stepper.move(-steps_per_mm * 400);
    
    uint32_t homingStartTime = millis();
    while (digitalRead(PIN_LIMIT_SWITCH) == HIGH) {
        stepper.run();
        
        // Safety timeout
        if (millis() - homingStartTime > 30000) {
            stepper.stop();
            return;
        }
        
        if (stepper.distanceToGo() == 0) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    stepper.stop();
    stepper.setCurrentPosition(0);
    delay(100);
    
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);
    stepper.moveTo(steps_per_mm * HOMING_OFFSET);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    stepper.setCurrentPosition(0);
    
    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(1000);

    isHomed = true;
}

// ---------------------------------------------
//  WEB / SETTINGS
// ---------------------------------------------
void handleWebRequest() {
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                   void *arg, uint8_t *data, size_t len) {
        
        if (type == WS_EVT_CONNECT) {
            currentTelemetry.wsConnected = true;
        }
        
        if (type == WS_EVT_DISCONNECT) {
            currentTelemetry.wsConnected = false;
        }
        
        if (type == WS_EVT_DATA) {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            
            if (info->final && info->index == 0 && info->len == len) {
                
                if (len == 15) {
                    blinkLED(3, 100);

                    uint8_t receivedCRC = data[14];
                    uint8_t calculatedCRC = crc8(data, 14);
                    
                    if (calculatedCRC != receivedCRC) {
                        return;
                    }

                    float zHeight;
                    float dispense;
                    uint16_t rpm;
                    uint16_t runTime;
                    uint16_t rampTime;
                    
                    memcpy(&zHeight, data + 0, sizeof(float));
                    memcpy(&dispense, data + 4, sizeof(float));
                    memcpy(&rpm, data + 8, sizeof(uint16_t));
                    memcpy(&runTime, data + 10, sizeof(uint16_t));
                    memcpy(&rampTime, data + 12, sizeof(uint16_t));
                    
                    updateSettings(zHeight, dispense, rpm, runTime, rampTime);
                }
                else if (len == 1) {
                    if (data[0] == 0x48) {  // 'H' - Home command
                        if (xSemaphoreTake(xMotionSemaphore, pdMS_TO_TICKS(100))) {
                            if (current_state == STATE_IDLE) {
                                current_state = STATE_HOMING;
                            }
                            xSemaphoreGive(xMotionSemaphore);
                        }
                    }
                    else if (data[0] == 0x53) {  // 'S' - Start
                        digitalWrite(STATUS_LED_PIN, HIGH);
                
                        if (!isHomed) {
                            blinkLED(5, 100);
                            return;
                        }
                        
                        if (xSemaphoreTake(xMotionSemaphore, pdMS_TO_TICKS(100))) {
                            if (current_state == STATE_IDLE) {
                                current_state = STATE_POSITIONING;
                                previous_state = STATE_IDLE;
                            } else {
                                blinkLED(5, 100);
                            }
                            xSemaphoreGive(xMotionSemaphore);
                        } else {
                            blinkLED(5, 100);
                        }
                    }
                    else if (data[0] == 0x58) {  // 'X' - Emergency stop
                        if (xSemaphoreTake(xMotionSemaphore, pdMS_TO_TICKS(100))) {
                            current_state = STATE_IDLE;
                            previous_state = STATE_IDLE;
                            xSemaphoreGive(xMotionSemaphore);
                        }
                        
                        if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(100))) {
                            motor_setRPM = 0;
                            xSemaphoreGive(xTelemetrySemaphore);
                        }
                        
                        dshotStop();
                    }
                }
            }
        }
    });
    
    server.addHandler(&ws);
    server.begin();
}

void updateSettings(float zHeight, float dispenseVol, int targetRPM, int runTime, int rampTime) {
    if (validateInputs(zHeight, dispenseVol, targetRPM, runTime, rampTime)) {
        currentSettings = {zHeight, dispenseVol, targetRPM, runTime, rampTime};
    }
}

void moveZAxis(float targetHeight) {
    stepper.moveTo(steps_per_mm * targetHeight);
}

void dispenseResist(float volumeUL) {
    int32_t steps = steps_per_ul * (volumeUL / 25.0);
    
    dispensor.setMaxSpeed(800); 
    dispensor.setAcceleration(400);  
    
    dispensor.move(-steps);
}

void runSpinCycle() {
    static state last_executed_state = STATE_IDLE;
    
    bool state_changed = (current_state != last_executed_state);
    
    switch(current_state) {
        case STATE_IDLE:
            digitalWrite(STATUS_LED_PIN, LOW);
            if (state_changed) {
                currentTelemetry.runStage = "Idle";
            }
            last_executed_state = STATE_IDLE;
            break;
        
        case STATE_HOMING:
            digitalWrite(STATUS_LED_PIN, HIGH);
            if (state_changed) {
                currentTelemetry.runStage = "Homing";
                homeZAxis();
                if (xSemaphoreTake(xStateSemaphore, pdMS_TO_TICKS(5))) {
                    current_state = STATE_IDLE;
                    xSemaphoreGive(xStateSemaphore);
                }
            }
            last_executed_state = STATE_HOMING;
            break;
            
        case STATE_POSITIONING:
            digitalWrite(STATUS_LED_PIN, HIGH);
            if (state_changed) {
                currentTelemetry.runStage = "Positioning";
                moveZAxis(currentSettings.zHeight);
            }
            
            if (stepper.distanceToGo() == 0) {
                if (xSemaphoreTake(xStateSemaphore, pdMS_TO_TICKS(5))) {
                    current_state = STATE_DISPENSING;
                    xSemaphoreGive(xStateSemaphore);
                }
            }
            last_executed_state = STATE_POSITIONING;
            break;
            
        case STATE_DISPENSING:
            digitalWrite(STATUS_LED_PIN, HIGH);
            if (state_changed) {
                currentTelemetry.runStage = "Dispensing";
                dispenseResist(currentSettings.dispenseVol);
            }
            
            if (dispensor.distanceToGo() == 0) {
                digitalWrite(PIN_ULN_IN1, LOW);
                digitalWrite(PIN_ULN_IN2, LOW);
                digitalWrite(PIN_ULN_IN3, LOW);
                digitalWrite(PIN_ULN_IN4, LOW);
                
                if (xSemaphoreTake(xStateSemaphore, pdMS_TO_TICKS(5))) {
                    current_state = STATE_RAMPUP;
                    xSemaphoreGive(xStateSemaphore);
                }
                time_ramping = 0;
            }
            last_executed_state = STATE_DISPENSING;
            break;
            
        case STATE_RAMPUP:
            digitalWrite(STATUS_LED_PIN, HIGH);
            if (state_changed) {
                currentTelemetry.runStage = "RampUp";
            }
            
            if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
                motor_setRPM = map(time_ramping, 0, currentSettings.rampTime, 
                                  0, currentSettings.targetRPM);
                motorPID.setPoint(motor_setRPM);
                xSemaphoreGive(xTelemetrySemaphore);
            }
            
            if (time_ramping >= currentSettings.rampTime) {
                if (xSemaphoreTake(xStateSemaphore, pdMS_TO_TICKS(5))) {
                    current_state = STATE_SPINNING;
                    xSemaphoreGive(xStateSemaphore);
                }
                time_coating = 0;
            }
            last_executed_state = STATE_RAMPUP;
            break;
            
        case STATE_SPINNING:
            digitalWrite(STATUS_LED_PIN, HIGH);
            if (state_changed) {
                currentTelemetry.runStage = "Spinning";
            }
            
            if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
                motor_setRPM = currentSettings.targetRPM;
                motorPID.setPoint(motor_setRPM);
                xSemaphoreGive(xTelemetrySemaphore);
            }
            
            if (time_coating >= currentSettings.runTime) {
                if (xSemaphoreTake(xStateSemaphore, pdMS_TO_TICKS(5))) {
                    current_state = STATE_RAMPDOWN;
                    xSemaphoreGive(xStateSemaphore);
                }
                time_ramping = 0;
            }
            last_executed_state = STATE_SPINNING;
            break;
            
        case STATE_RAMPDOWN:
            digitalWrite(STATUS_LED_PIN, HIGH);
            if (state_changed) {
                currentTelemetry.runStage = "RampDown";
            }
            
            if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
                motor_setRPM = map(time_ramping, 0, currentSettings.rampTime, 
                                  currentSettings.targetRPM, 0);
                motorPID.setPoint(motor_setRPM);
                xSemaphoreGive(xTelemetrySemaphore);
            }
            
            if (time_ramping >= currentSettings.rampTime) {
                if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
                    motor_setRPM = 0;
                    motorPID.setPoint(motor_setRPM);
                    xSemaphoreGive(xTelemetrySemaphore);
                }
                if (xSemaphoreTake(xStateSemaphore, pdMS_TO_TICKS(5))) {
                    current_state = STATE_IDLE;
                    xSemaphoreGive(xStateSemaphore);
                }
            }
            last_executed_state = STATE_RAMPDOWN;
            break;
    }
}

void sendStatusUpdate() {
    currentTelemetry.currentZ = stepper.currentPosition() / (float)steps_per_mm;
    
    uint16_t rpmToSend = (uint16_t)currentTelemetry.currentRPM;
    
    uint8_t packet[12];
    memcpy(packet, &currentTelemetry.currentZ, 4);
    memcpy(packet + 4, &rpmToSend, 2);
    packet[6] = mapStageToByte(currentTelemetry.runStage);
    packet[7] = currentTelemetry.wsConnected ? 1 : 0;
    packet[8] = isHomed ? 1 : 0;
    packet[9] = 0;
    packet[10] = 0;
    
    packet[11] = crc8(packet, 11);
    
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 1000) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        lastBlink = millis();
    }
    
    ws.binaryAll(packet, sizeof(packet));
}

// ---------------------------------------------
//  TFT DISPLAY
// ---------------------------------------------
void updateDisplay() {
    static String lastStage = "";
    static int lastRPM = -1;
    static float lastZ = -999;
    static bool lastHomed = false;
    
    bool needsUpdate = (currentTelemetry.runStage != lastStage) ||
                       (currentTelemetry.currentRPM != lastRPM) ||
                       (abs(currentTelemetry.currentZ - lastZ) > 0.1) ||
                       (isHomed != lastHomed);
    
    if (!needsUpdate) return;
    
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("SPIN COATER");
    
    tft.setTextSize(1);
    tft.setCursor(10, 35);
    if (isHomed) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("HOMED");
    } else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.println("NOT HOMED");
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // State
    tft.setTextSize(2);
    tft.setCursor(10, 55);
    tft.print("State: ");
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println(currentTelemetry.runStage);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // RPM
    tft.setTextSize(3);
    tft.setCursor(10, 90);
    tft.print(currentTelemetry.currentRPM);
    tft.setTextSize(2);
    tft.println(" RPM");
    
    // Z Position
    tft.setTextSize(2);
    tft.setCursor(10, 130);
    tft.print("Z: ");
    tft.print(currentTelemetry.currentZ, 1);
    tft.println(" mm");
    
    // Settings
    tft.setTextSize(1);
    tft.setCursor(10, 160);
    tft.println("Settings:");
    tft.setCursor(10, 175);
    tft.print("Target: ");
    tft.print(currentSettings.targetRPM);
    tft.println(" RPM");
    tft.setCursor(10, 190);
    tft.print("Time: ");
    tft.print(currentSettings.runTime / 1000);
    tft.println(" s");
    tft.setCursor(10, 205);
    tft.print("Vol: ");
    tft.print(currentSettings.dispenseVol, 0);
    tft.println(" uL");
    
    lastStage = currentTelemetry.runStage;
    lastRPM = currentTelemetry.currentRPM;
    lastZ = currentTelemetry.currentZ;
    lastHomed = isHomed;
}

// ---------------------------------------------
//  UTILITY FUNCTIONS
// ---------------------------------------------
bool validateInputs(float zHeight, float dispenseVol, int targetRPM, int runTime, int rampTime) {
    return (zHeight >= 0 && zHeight <= Z_HEIGHT_MAX) &&
           (dispenseVol >= 25 && dispenseVol <= 1000) &&
           (targetRPM >= MIN_COAT_SPEED && targetRPM <= MAX_COAT_SPEED) &&
           (runTime >= MIN_COAT_TIME && runTime <= MAX_COAT_TIME) &&
           (rampTime >= MIN_RAMPUP_TIME && rampTime <= MAX_RAMPDOWN_TIME);
}

uint8_t mapStageToByte(String stage) {
    if (stage == "Idle") return 0;
    if (stage == "Homing") return 1;        
    if (stage == "Positioning") return 2;   
    if (stage == "Dispensing") return 3;    
    if (stage == "RampUp") return 4;        
    if (stage == "RampDown") return 5;     
    if (stage == "Spinning") return 6;      
    return 255;
}

uint8_t crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        uint8_t extract = *data++;
        for (uint8_t tempI = 8; tempI; tempI--) {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) crc ^= 0x8C;
            extract >>= 1;
        }
    }
    return crc;
}

// ---------------------------------------------
//  FreeRTOS FUNCTIONS
// ---------------------------------------------
void taskMotionControl(void * pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        stepper.run();
        dispensor.run();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void taskSpinControl(void * pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        if (xSemaphoreTake(xMotionSemaphore, pdMS_TO_TICKS(10))) {
            runSpinCycle();
            xSemaphoreGive(xMotionSemaphore);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void taskTelemetry(void * pvParameters) { 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
    
    bool motorStarting = false;
    uint32_t motorStartTime = 0;
    const uint16_t STARTUP_THROTTLE = 250;    
    const uint32_t STARTUP_DURATION_MS = 800;   
    const uint16_t MIN_OPERATING_RPM = 200;
    
    uint32_t debugCounter = 0;
    
    for (;;) {
        if (debugCounter++ % 100 == 0) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        }
        
        // TELEMETRY PARSING
        if (TelSerial.available() >= 10) {
            uint8_t buffer[10];
            TelSerial.readBytes(buffer, 10);
            
            uint8_t crc = crc8(buffer, 9);
            if (crc == buffer[9]) {
                temperature = buffer[0];
                voltage = (buffer[1] << 8 | buffer[2]) / 100.0;
                current = (buffer[3] << 8 | buffer[4]) / 100.0;
                erpm = (buffer[5] << 8 | buffer[6]) * 100;
                
                uint32_t local_rpm = erpm / MOTOR_POLE_PAIRS;
                
                if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
                    rpm = local_rpm;
                    currentTelemetry.currentRPM = local_rpm;
                    xSemaphoreGive(xTelemetrySemaphore);
                }
                
                digitalWrite(STATUS_LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(2));
                digitalWrite(STATUS_LED_PIN, LOW);
            }
        } else {
            // If no telemetry, still update with fake values for testing
            // This will help us see if the problem is ESC telemetry or data transmission
            if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
                // Use motor_setRPM as fake currentRPM for testing
                currentTelemetry.currentRPM = (int)(motor_setRPM * 0.95); // Simulate 95% accuracy
                xSemaphoreGive(xTelemetrySemaphore);
            }
        }
        
        double local_setRPM;
        uint32_t local_actualRPM;
        
        if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(5))) {
            local_setRPM = motor_setRPM;
            local_actualRPM = rpm;
            xSemaphoreGive(xTelemetrySemaphore);
        }
        
        if (local_setRPM > MIN_OPERATING_RPM) {
            if (!motorStarting && local_actualRPM < MIN_OPERATING_RPM) {
                motorStarting = true;
                motorStartTime = millis();
            }
            
            if (motorStarting) {
                uint32_t elapsed = millis() - motorStartTime;
                
                if (local_actualRPM >= MIN_OPERATING_RPM) {
                    motorStarting = false;
                    motorPID.start(); 
                    
                    pid_output = motorPID.compute(local_actualRPM);
                    if (pid_output < 150) pid_output = 150;
                    dshotOutput((uint16_t)pid_output, true);
                    
                } else if (elapsed < STARTUP_DURATION_MS) {
                    dshotOutput(STARTUP_THROTTLE, true);
                    
                } else {
                    uint16_t boosted = STARTUP_THROTTLE + 150;
                    if (boosted > 500) boosted = 500;
                    dshotOutput(boosted, true);
                }
            }
            else {
                pid_output = motorPID.compute(local_actualRPM);
                
                if (pid_output < 120) {
                    pid_output = 120;
                }
                
                dshotOutput((uint16_t)pid_output, true);
            }
            
        } else {
            motorStarting = false;
            dshotOutput(0, false);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    } 
}

void taskDisplay(void * pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xDisplaySemaphore, pdMS_TO_TICKS(20))) {
            updateDisplay();
            xSemaphoreGive(xDisplaySemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    } 
}

void taskSafety(void * pvParameters) {
    for (;;) {
        if (temperature > 80) {
            if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(10))) {
                motor_setRPM = 0;
                xSemaphoreGive(xTelemetrySemaphore);
            }
            if (xSemaphoreTake(xMotionSemaphore, pdMS_TO_TICKS(10))) {
                current_state = STATE_IDLE;
                xSemaphoreGive(xMotionSemaphore);
            }
            dshotStop();
        }
        
        if (voltage < 10.0 && voltage > 0) {
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

void taskWebSocket(void * pvParameters) {
    for (;;) {
        ws.cleanupClients();
        
        if (xSemaphoreTake(xTelemetrySemaphore, pdMS_TO_TICKS(10))) {
            sendStatusUpdate();
            xSemaphoreGive(xTelemetrySemaphore);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------
//  SETUP / LOOP
// ---------------------------------------------
void setup() {
    Serial.begin(115200);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    pinMode(PIN_TMC_EN, OUTPUT);
    digitalWrite(PIN_TMC_EN, LOW);

    pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
    
    pinMode(PIN_ULN_IN1, OUTPUT);
    pinMode(PIN_ULN_IN2, OUTPUT);
    pinMode(PIN_ULN_IN3, OUTPUT);
    pinMode(PIN_ULN_IN4, OUTPUT);
    digitalWrite(PIN_ULN_IN1, LOW);
    digitalWrite(PIN_ULN_IN2, LOW);
    digitalWrite(PIN_ULN_IN3, LOW);
    digitalWrite(PIN_ULN_IN4, LOW);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    TMCSerial.begin(115200, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
    driver.begin();
    driver.rms_current(800);
    driver.microsteps(HALFSTEP);
    driver.pwm_autoscale(true);

    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(1000);

    dispensor.setMaxSpeed(800);    
    dispensor.setAcceleration(400); 
    dispensor.setSpeed(600);  

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(60, 100);
    tft.println("SPIN COATER");
    tft.setTextSize(1);
    tft.setCursor(80, 130);
    tft.println("Initializing...");

    currentSettings.zHeight = 50.0;
    currentSettings.dispenseVol = 100.0;
    currentSettings.targetRPM = 3000;
    currentSettings.runTime = 30000;
    currentSettings.rampTime = 3000;
    
    currentTelemetry.currentZ = 0.0;
    currentTelemetry.currentRPM = 0;
    currentTelemetry.runStage = "Idle";
    currentTelemetry.wsConnected = false;

    TelSerial.begin(115200, SERIAL_8N1, PIN_ESC_RX, -1);
    motorPID.setPoint(motor_setRPM);
    motorPID.setOutputRange(pid_out_min, pid_out_max);
    motorPID.start();

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    handleWebRequest();

    initDshot();
    initESC();

    xTelemetrySemaphore = xSemaphoreCreateMutex();
    xMotionSemaphore    = xSemaphoreCreateMutex();
    xStateSemaphore     = xSemaphoreCreateMutex();
    xDisplaySemaphore   = xSemaphoreCreateMutex(); 

    xTaskCreatePinnedToCore(taskSpinControl, "Spin", TASK_STACK_DEFAULT, NULL, TASK_PRIORITY_HIGH, NULL, 0); 
    xTaskCreatePinnedToCore(taskTelemetry, "Telemetry", TASK_STACK_DEFAULT, NULL, TASK_PRIORITY_MED, NULL, 0);
    xTaskCreatePinnedToCore(taskSafety, "Safety", TASK_STACK_DEFAULT, NULL, TASK_PRIORITY_HIGH, NULL, 0); 
    xTaskCreatePinnedToCore(taskMotionControl, "Motion", TASK_STACK_DEFAULT, NULL, TASK_PRIORITY_HIGH, NULL, 1);
    xTaskCreatePinnedToCore(taskWebSocket, "WebSocket", TASK_STACK_DEFAULT, NULL, TASK_PRIORITY_LOW, NULL, 1);
    xTaskCreatePinnedToCore(taskDisplay, "Display", TASK_STACK_DEFAULT, NULL, TASK_PRIORITY_LOW, NULL, 1);  

    blinkLED(2, 200);
    tft.fillScreen(TFT_BLACK);
    updateDisplay();
}

void loop() {}