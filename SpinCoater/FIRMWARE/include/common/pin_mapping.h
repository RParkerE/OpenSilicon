#pragma once

#define TFT_CS              15
#define TFT_DC              22
#define TFT_RST             4
#define TFT_MOSI            23
#define TFT_SCLK            18

#define ESC_PWM_PIN         13
// NOTE: No separate ESC telemetry/RX wire. RPM comes back over ESC_PWM_PIN via
// bidirectional DShot. (The old ESC_RX_PIN=9 was a flash pin and never worked.)
#define STATUS_LED_PIN      2

#define Z_STEP_PIN          25
#define Z_DIR_PIN           26
#define Z_ENABLE_PIN        27
#define Z_RX_PIN            16
#define Z_TX_PIN            17
#define Z_LIMIT_SWITCH_PIN  19

#define PUMP_IN1            32
#define PUMP_IN2            21
#define PUMP_IN3            14
#define PUMP_IN4            33