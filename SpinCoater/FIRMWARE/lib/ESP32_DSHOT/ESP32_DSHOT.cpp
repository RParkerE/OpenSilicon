#include "esp32-hal.h"
#if defined ARDUINO_ARCH_ESP32
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

#include "ESP32_DSHOT.h"
#include <Arduino.h>

#define RMT_CHANNELS 8
DSHOT *rx_channel_map[RMT_CHANNELS] = {NULL};
DSHOT *tx_channel_map[RMT_CHANNELS] = {NULL};
uint8_t used_rmt_channels = 0;

#include <hal/rmt_ll.h>
#define RMT_RX_CHANNEL_ENCODING_START (SOC_RMT_CHANNELS_PER_GROUP-SOC_RMT_TX_CANDIDATES_PER_GROUP)
#define RMT_ENCODE_RX_CHANNEL(decode_chan) ((decode_chan + RMT_RX_CHANNEL_ENCODING_START))

rmt_isr_handle_t xHandler = NULL;
void IRAM_ATTR rmt_isr_handler(void* arg);


bool DSHOT::begin(int pin, dshot_mode_t dshot_mode) {
  setMode(dshot_mode);

  //check for enough free RMT channels
  int ch_needed = (is_bidirectional ? 2 : 1);
  if(used_rmt_channels + ch_needed > RMT_CHANNELS) return false;

  //assign channels
  if(is_bidirectional) {
    rx_channel = (rmt_channel_t)used_rmt_channels;
    rx_channel_map[used_rmt_channels] = this;
    used_rmt_channels++;
  }
  tx_channel = (rmt_channel_t)used_rmt_channels;
  tx_channel_map[used_rmt_channels] = this;
  used_rmt_channels++;

  //set pin
  this->pin = (gpio_num_t)pin;

  //setup RMT
  rm_setup();

  //get ESC going by setting line low for one second
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  //delay(1000);

  return true;
}


void DSHOT::set(float throttle, bool telem) {
  set(throttle*2000, telem);
}


void DSHOT::set(int throttle, bool telem) {
  rm_send(throttle+48, telem);
}


void DSHOT::cmd(int cmd, bool telem) {
  if(cmd>=0 && cmd<48) rm_send(cmd, telem);
}


void DSHOT::arm() {
  //arm procedure: pull pin low, motor stop, throttle 0, motor stop (each step 0.5 sec)
  // FVT LittleBee 20A - BlueJay v0.19 -> 300, 600, 300bidir, 600bidir work
  // Hobbywing Xrotor Micro 60A 4in1 - BLHeli32 v32.9 -> 300 works, 300bidir sometimes works.... (but same problem in BetaFlight motor tester, is this ESC related???)

  //pull ESC line low (0.5 sec) - needed for BlueJay bidirectional
  for(int ch=0;ch<RMT_CHANNELS;ch++) {
    if(tx_channel_map[ch]) {
      int pin = tx_channel_map[ch]->get_pin();
      pinMode(pin,OUTPUT);
      digitalWrite(pin,LOW);
    }
  }
  delay(500);

  //send motor stop (0.5 sec)
  for(int i=0;i<500;i++) {
    for(int ch=0;ch<RMT_CHANNELS;ch++) {
      if(tx_channel_map[ch]) {
        tx_channel_map[ch]->cmd(DSHOT_CMD_MOTOR_STOP);
      }
    }
    delay(1);
  }

  //send zero thottle (0.5 sec)
  for(int i=0;i<500;i++) {
    for(int ch=0;ch<RMT_CHANNELS;ch++) {
      if(tx_channel_map[ch]) {
        tx_channel_map[ch]->set(0);
      }
    }
    delay(1);
  }

  //send motor stop (0.5 sec)
  for(int i=0;i<500;i++) {
    for(int ch=0;ch<RMT_CHANNELS;ch++) {
      if(tx_channel_map[ch]) {
        tx_channel_map[ch]->cmd(DSHOT_CMD_MOTOR_STOP);
      }
    }
    delay(1);
  }


}

void DSHOT::setMode(dshot_mode_t dshot_mode) {
    switch (dshot_mode)
    {
    case DSHOT150_BIDIR:
        is_bidirectional = true;
        ticks_per_bit =   67; //6.667us
        ticks_zero_high = 25; //2.500
        ticks_one_high =  50; //5.000
        bidir_bit_ns =  5333; //5.333us = 4/5 * 6.667
        break;
    case DSHOT150:
        is_bidirectional = false;
        ticks_per_bit =   67; //6.667us
        ticks_zero_high = 25; //2.500
        ticks_one_high =  50; //5.000
        bidir_bit_ns =  5333; //5.333us = 4/5 * 6.667
        break;
    case DSHOT300_BIDIR:
        is_bidirectional = true;
        ticks_per_bit =   33; //3.333
        ticks_zero_high = 12; //1.250
        ticks_one_high =  25; //2.500
        bidir_bit_ns =  2666;
        break;
    case DSHOT300:
        is_bidirectional = false;
        ticks_per_bit =   33; //3.333
        ticks_zero_high = 12; //1.250
        ticks_one_high =  25; //2.500
        bidir_bit_ns =  2666;
        break;
    case DSHOT600_BIDIR:
        is_bidirectional = true;
        ticks_per_bit =   17; //1.667
        ticks_zero_high =  6; //0.625
        ticks_one_high =  12; //1.250
        bidir_bit_ns =  1333;        
        break;
    case DSHOT600:
        is_bidirectional = false;
        ticks_per_bit =   17; //1.667
        ticks_zero_high =  6; //0.625
        ticks_one_high =  12; //1.250
        bidir_bit_ns =  1333;        
        break;
    case DSHOT1200_BIDIR:
        is_bidirectional = true;
        ticks_per_bit =    8; //0.833
        ticks_zero_high =  3; //0.313
        ticks_one_high =   6; //0.625
        bidir_bit_ns =   667;
        break;
    case DSHOT1200:
        is_bidirectional = false;
        ticks_per_bit =    8; //0.833
        ticks_zero_high =  3; //0.313
        ticks_one_high =   6; //0.625
        bidir_bit_ns =   667;   
        break;

    // Default to DSHOT300
    default:
        is_bidirectional = false;
        ticks_per_bit =   33; //3.333
        ticks_zero_high = 12; //1.250
        ticks_one_high =  25; //2.500
        bidir_bit_ns =  2666;        
        break;
    }

    //interval in us: 16 DSHOT bits, 30us pause, 21 telem bits, 30us pause. Plus 10us extra.
    interval_us = (16*ticks_per_bit*100 + 30000 + (is_bidirectional ? 0 : 21 * bidir_bit_ns + 30000 ))/1000 + 10;
}


uint16_t DSHOT::calcCRC(uint16_t packet)
{
  packet >>= 4;
  uint16_t crc = packet ^ (packet >> 4) ^ (packet >> 8);
  if (is_bidirectional) crc = ~crc;
  crc &= 0x0F;
  return crc;
}


void DSHOT::calcItems(rmt_item32_t *item, uint16_t cmd, bool telem)
{
  if (cmd > 2047) {
      cmd = 2047;
  }

  uint16_t packet = (cmd << 5) | ((telem ? 1 : 0)<<4);
  packet |= calcCRC(packet);

  for (int i = 0; i < 16; i++, packet <<= 1) {
      if (packet & 0b1000000000000000) {
          // Set RMT item for bit=1
          item[i].duration0 = ticks_one_high;
          item[i].duration1 = ticks_per_bit - ticks_one_high;
      } else {
          // Set RMT item for bit=0
          item[i].duration0 = ticks_zero_high;
          item[i].duration1 = ticks_per_bit - ticks_zero_high;
      }

      // Set level of RMT item
      if (is_bidirectional) {
        item[i].level0 = 0;
        item[i].level1 = 1;
      }else{
        item[i].level0 = 1;
        item[i].level1 = 0;
      }
  }

  //set last item duration = 0
  item[15].duration1 = 0;

  //for(int i=0;i<16;i++) Serial.printf("item[%d] = %d %d %d %d\n",i,(int)item[i].duration0,(int)item[i].level0,(int)item[i].duration1,(int)item[i].level1);
}


IRAM_ATTR uint8_t DSHOT::revertMapping(uint16_t value) {
  switch(value) {
    case 0x19: return 0x00;
    case 0x1B: return 0x01;
    case 0x12: return 0x02;
    case 0x13: return 0x03;
    case 0x1D: return 0x04;
    case 0x15: return 0x05;
    case 0x16: return 0x06;
    case 0x17: return 0x07;
    case 0x1A: return 0x08;
    case 0x09: return 0x09;
    case 0x0A: return 0x0A;
    case 0x0B: return 0x0B;
    case 0x1E: return 0x0C;
    case 0x0D: return 0x0D;
    case 0x0E: return 0x0E;
    case 0x0F: return 0x0F;
  }

  return 0xFF;
}


//===========================================================================================
// RMT specific code
//===========================================================================================

void DSHOT::rm_setup_tx() {
  tx_conf.rmt_mode = RMT_MODE_TX;

  tx_conf.channel = tx_channel;
  tx_conf.gpio_num = pin;
  tx_conf.mem_block_num = 1;
  tx_conf.clk_div = 8;

  tx_conf.tx_config.loop_en = false;
  tx_conf.tx_config.carrier_en = false;
  tx_conf.tx_config.idle_output_en = true;

  rmt_config(&tx_conf);

  //set idle and pull
  if(is_bidirectional) {
    rmt_set_idle_level(tx_channel, true, RMT_IDLE_LEVEL_HIGH); //idle = high
  }else{
    rmt_set_idle_level(tx_channel, true, RMT_IDLE_LEVEL_LOW); //idle = low
  }
}


void DSHOT::rm_setup_rx() {  
  rx_conf.rmt_mode = RMT_MODE_RX;

  rx_conf.channel = rx_channel;
  rx_conf.gpio_num = pin;
  rx_conf.mem_block_num = 1;
  rx_conf.clk_div = 8;

  rx_conf.rx_config.idle_threshold = 350; //35us
  rx_conf.rx_config.filter_en = false;
  rx_conf.rx_config.filter_ticks_thresh = 0;  

  rmt_config(&rx_conf);
}


void DSHOT::rm_setup() {
  if(is_bidirectional) {
    rm_setup_rx();
  }
  
  rm_setup_tx();

  //enable interrupts
  rmt_isr_register(rmt_isr_handler, NULL, ESP_INTR_FLAG_LEVEL1, &xHandler); 
}


void DSHOT::rm_send(uint16_t cmd, bool telem)
{
  rmt_item32_t item[16];

  calcItems(item, cmd, telem);

  //copy items to RMT channel
  rmt_fill_tx_items(tx_channel, item, 16, 0);

  //switch to TX config
  if(is_bidirectional) {
    digitalWrite(pin, HIGH);
    rmt_set_tx_intr_en(tx_channel, true);
  }
  rmt_set_gpio(tx_channel, RMT_MODE_TX, pin, false);

  //send the dshot pulses to the ESC
  rmt_tx_start(tx_channel, true);
}


//handle transmission completed interrupt
IRAM_ATTR void DSHOT::rm_tx_end_handler() {
  if(is_bidirectional) {
    //switch RMT to receiving
    rmt_set_gpio(rx_channel, RMT_MODE_RX, pin, false);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY); //enable pullup
    rmt_rx_start(rx_channel, true);
  }
}


IRAM_ATTR int DSHOT::decode_bidir(uint32_t value) {
  uint16_t newValue;
  uint16_t mapped = 0x00;
  uint8_t leftShift = 0;

  value ^= (value >> 1); //21 bit -> 20 bit

  for(int i = 0; i < 20; i += 5) {
    newValue = revertMapping(((value >> i) & 0x1F));
    mapped |= newValue << leftShift;
    leftShift += 4;
  }

  int crc = (~((mapped >> 4) ^ (mapped >> 8) ^ (mapped >> 12) )) & 0x0F;

  if(crc != (mapped & 0x0F) ) {
    return -1;
  }
  return mapped>>4;
}














//handle receive completed interrupt
IRAM_ATTR void DSHOT::rm_rx_end_handler() {
  
  //get access to the received items
  rmt_ll_rx_enable(&RMT, rx_channel, false);
  rmt_ll_rx_set_mem_owner(&RMT, rx_channel, RMT_MEM_OWNER_TX);
  volatile rmt_item32_t* item = NULL;
  item = RMTMEM.chan[RMT_ENCODE_RX_CHANNEL(rx_channel)].data32;

  //decode telemetry datas
  if(item) {
    if(item[0].duration0 > 0 && item[0].level0 == 0) { //rmt items should start with a '0'
      //update statistics
      telem_cnt++;

      //get shifted gcr value from rmt durations
      int r = 0;
      int bcnt = 0;
      for(int i=0;i<21;i++) { //max 21 durations (1 per bit)
        int bit,bits;
        if(item[i].duration0==0) break;
        bit = item[i].level0;
        bits = ((int)item[i].duration0 * 100 + bidir_bit_ns/2)/bidir_bit_ns;
        bcnt += bits;
        for(int j=0;j<bits;j++) {
          r = (r << 1) | bit;
        }
        if(item[i].duration1==0) break;
        bit = item[i].level1;
        bits = ((int)item[i].duration1 * 100 + bidir_bit_ns/2)/bidir_bit_ns;
        bcnt += bits;
        for(int j=0;j<bits;j++) {
          r = (r << 1) | bit;
        }      
      }
      
      if(bcnt<=21) {
        for(int i=bcnt;i<21;i++) r = (r << 1) | 1; //append 1's to reveive 21 bits

        //decode shifted gcr value
        r = decode_bidir(r);
        if(r>=0) {
          //update statistics
          telem_ok_cnt++;
          //decode telemetry value
          if(r & 0x100) {
            if(r == 0xFFF) {
              erpm_us = 0;
            }else{
              erpm_us = (r & 0x1ff) << (r >> 9); //erpm period in us
            }
          }else{
            telem[ (r >> 9) & 0x7 ] = r & 0xff;
          }
        }
      }
    }
  }

  //return item access to RMT receiver
  rmt_ll_rx_set_mem_owner(&RMT, rx_channel, RMT_MEM_OWNER_RX);
}


//RMT Interrupt Handler
IRAM_ATTR void rmt_isr_handler(void* arg){
  uint32_t status;

  //TX end interrupt 
  status = rmt_ll_get_tx_end_interrupt_status(&RMT);
  for(int ch=0;ch<RMT_CHANNELS;ch++) {
    if (status & (1<<ch) ) {
      if (tx_channel_map[ch]) tx_channel_map[ch]->rm_tx_end_handler();
      rmt_ll_clear_tx_end_interrupt(&RMT, ch);
    }
  }

  //RX end interrupt
  status = rmt_ll_get_rx_end_interrupt_status(&RMT);
  for(int ch=0;ch<RMT_CHANNELS;ch++) {
    if (status & (1<<ch) ) {
      if (rx_channel_map[ch]) rx_channel_map[ch]->rm_rx_end_handler();
      rmt_ll_clear_rx_end_interrupt(&RMT, ch);
    }
  }
}

#endif //#if defined ARDUINO_ARCH_ESP32
