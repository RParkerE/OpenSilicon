/*==========================================================================================
ESP32_DSHOT - ESP32 Arduino library for bidirectional DSHOT

Tested on ESP32 with Arduino-ESP32 v2.0.14 (ESP-IDF v4.4), might work on other ESP32 variants with modifications.

MIT License

Copyright (c) 2024 qqqlab - https://github.com/qqqlab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#pragma once

#include <driver/rmt.h>

class DSHOT {
public:

  // Enumeration for the DShot mode
  typedef enum dshot_mode_e
  {
    DSHOT150,
    DSHOT150_BIDIR,
    DSHOT300,
    DSHOT300_BIDIR,
    DSHOT600,
    DSHOT600_BIDIR,
    DSHOT1200,
    DSHOT1200_BIDIR
  } dshot_mode_t;

  // enum for telem[]
  typedef enum dshot_telem_e
  {
    TELEM_TEMPERATURE = 1, //0x200: Temperature range (degree Celsius)
    TELEM_VOLTAGE = 2, //0x400: Voltage range (0-63,75V step 0,25V)
    TELEM_CURRENT = 3, //0x600: Current range (0-255A step 1A)
    TELEM_DEBUG1 = 4, //0x800: Debug 1 value
    TELEM_DEBUG2 = 5, //0xA00: Debug 2 value
    TELEM_DEBUG3 = 6, //0xC00: Debug 3 value
    TELEM_STATE = 7  //0xE00: State / events
  } dshot_telem_t;

  typedef enum dshot_cmd_e
  {
    DSHOT_CMD_MOTOR_STOP = 0,               // Currently not implemented - STOP Motors
    DSHOT_CMD_BEEP1 = 1,                    // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP2 = 2,                    // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP3 = 3,                    // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP4 = 4,                    // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP5 = 5,                    // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_ESC_INFO = 6,                 // Currently not implemented
    DSHOT_CMD_SPIN_DIRECTION_1 = 7,         // bluejay=CMD_DIRECTION_NORMAL Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_2 = 8,         // bluejay=CMD_DIRECTION_REVERSE Need 6x, no wait required
    DSHOT_CMD_3D_MODE_OFF = 9,              // bluejay=CMD_BIDIR_OFF Need 6x, no wait required
    DSHOT_CMD_3D_MODE_ON = 10,              // bluejay=CMD_BIDIR_ON Need 6x, no wait required
    DSHOT_CMD_SETTINGS_REQUEST = 11,        // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS = 12,           // Need 6x, wait at least 12ms before next command
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE = 13,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE = 14,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,   // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21, // Need 6x, no wait required
    DSHOT_CMD_LED0_ON = 15,                 // BLHeli32 only
    DSHOT_CMD_LED1_ON = 16,                 // BLHeli32 only
    DSHOT_CMD_LED2_ON = 17,                 // BLHeli32 only
    DSHOT_CMD_LED3_ON = 18,                 // BLHeli32 only
    DSHOT_CMD_LED0_OFF = 19,                // BLHeli32 only
    DSHOT_CMD_LED1_OFF = 20,                // BLHeli32 only
    DSHOT_CMD_LED2_OFF = 21,                // BLHeli32 only
    DSHOT_CMD_LED3_OFF = 22,                // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,// KISS audio Stream mode on/off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,      // KISS silent Mode on/off
      DSHOT_CMD_MAX = 47
  } dshot_cmd_t;

  bool begin(int pin, dshot_mode_t dshot_mode);
  static void arm(); //reset all ESCs by pulling line low for one second and then sending 0 throttle for one second
  void set(float throttle, bool telem = false); //set throttle 0.0 to 1.0
  void set(int throttle, bool telem = false); //set throttle 0 to 2000
  void cmd(int cmd, bool telem = false); //send command 0 to 47

  int get_pin() {return pin;}

  //bidirectional dshot telemetry
  uint16_t erpm_us; //erpm period in us, or 0 if not available
  uint8_t telem[8] = {0}; //telemetry data
  uint32_t telem_cnt = 0; //number of telemetry packets received
  uint32_t telem_ok_cnt = 0; //number of OK telemetry packets received

private:
  //default DSHOT300
  bool is_bidirectional = false;
  int ticks_per_bit =   33; //3.333us
  int ticks_zero_high = 12; //1.250
  int ticks_one_high =  25; //2.500
  int bidir_bit_ns = 2666; //bit period for bidir telem reply. DSHOT300 = 3333 * 4/5 = 2666 ns
  int interval_us = 0;

  void setMode(dshot_mode_t dshot_mode);
  uint16_t calcCRC(uint16_t packet);
  void calcItems(rmt_item32_t *item, uint16_t cmd, bool telem = false);
  static uint8_t revertMapping(uint16_t value);
  static int decode_bidir(uint32_t value);

  rmt_channel_t rx_channel;
  rmt_channel_t tx_channel;
  rmt_config_t rx_conf;
  rmt_config_t tx_conf;
  gpio_num_t pin;

  void rm_setup_tx();
  void rm_setup_rx();
  void rm_setup();
  void rm_send(uint16_t cmd, bool telem = false);

public:
  void rm_tx_end_handler(); //handle transmission completed interrupt
  void rm_rx_end_handler(); //handle receive completed interrupt
};
