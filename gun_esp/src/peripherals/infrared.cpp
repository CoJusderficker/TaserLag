#include "infrared.h"
#include "config.h"

rmt_config_t rmt_cfg;
const rmt_item32_t rmt_item_high = {{{299, 1, 1, 0}}};
const rmt_item32_t rmt_item_low = {{{299, 0, 1, 0}}};

uint32_t changes_1[32] = {};
size_t changes_1_size = 0;

// log changes of sens pin
void IRAM_ATTR isr_sens_1() {
  changes_1[changes_1_size] = micros();
  changes_1_size++;
}

void infrared::init() {
  attachInterrupt(PIN_IR_RX, isr_sens_1, CHANGE);

  // configure rmt and install driver
  rmt_cfg.channel = RMT_CHANNEL_0;
  rmt_cfg.rmt_mode = RMT_MODE_TX;
  rmt_cfg.gpio_num = PIN_LED_IR;
  rmt_cfg.mem_block_num = 1;
  rmt_cfg.clk_div = 80; // 1 unit = 1 usec

  rmt_cfg.tx_config.carrier_duty_percent = 50;
  rmt_cfg.tx_config.carrier_en = true;
  rmt_cfg.tx_config.loop_en = false;
  rmt_cfg.tx_config.carrier_freq_hz = 38000;
  rmt_cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  rmt_cfg.tx_config.idle_output_en = false;

  rmt_config(&rmt_cfg);
  rmt_driver_install(rmt_cfg.channel, 16, 0);
}

uint16_t infrared::generate_ir_code(uint8_t player_id) {
  // returns message for shooting.
  // P: playerid, C: checksum
  // structure: 0b 1PPP PPPP PCCC CC01 (starts with 1 and ends with 01)

  // first, make frame:
  uint16_t ircode = 0b1000000000000001;

  // append playerid with space for 5 bits cs and 2 bits ending
  ircode += player_id << 7;

  // generate checksum
  uint8_t checksum = 0;
  for (int i = 0; i < 16; i++) {

    if (bitRead(ircode, i)) {
      checksum++;
    }
  }

  // write checksum in ircode
  ircode += checksum << 2;

  Serial.print("Ir Code Generated: ");
  Serial.println(ircode, BIN);

  return ircode;
}

// turn uint16 to array of rmt items.
rmt_item32_t *ircode_2_rmt(uint16_t ircode) {
  rmt_item32_t items[16];
  for (int i = 0; i < 16; i++) {

    if (bitRead(ircode, i)) {
      items[15 - i] = rmt_item_high;
    } else {
      items[15 - i] = rmt_item_low;
    }
  }
  return items;
}

uint8_t countBits(uint16_t n) {
  uint8_t count = 0;
  while (n) {
    n &= (n - 1); // delete least-significant bit
    count++;
  }
  return count;
}

// analyses IR buffer and determines, whether i was shot
void analyzeBuffer(uint32_t *buffer, size_t *buffer_size) {

  // if buffer is not empty and first entry is older than 10ms, analyse
  if (buffer[0] && micros() - buffer[0] >= 10000) {

    // 1st: check for time diff under 150us -> error
    for (size_t i = 1; i < *buffer_size; i++) {
      if (buffer[i] - buffer[i - 1] < 150) {
        Serial.println("Too tight timing.");
        return;
      }
    }
    Serial.println("timing ok.");

    // generate binary sequence from timestamps
    uint16_t sequence;
    uint8_t bit = 0;
    bool state = true;

    for (size_t i = 1; i < *buffer_size; i++) { // read out data
      int diff = buffer[i] - buffer[i - 1];
      int nearest_multiple = round(
          diff /
          300.0); // find nearest multiple to 300usec -> append that many digits

      for (int j = 0; j < nearest_multiple; j++) { // append
        if (bit < 16) {                            // prevent overflow

          bitWrite(sequence, 15 - bit, state);
          bit++;
        }
      }
      state = !state;
    }
    Serial.print("Received Ir Sequence: ");
    Serial.println(sequence, BIN);

    Serial.println("Clear buffer...");
    memset(buffer, 0, (*buffer_size) * sizeof(uint32_t)); // clear array
    *buffer_size = 0;

    // validate checksum

    uint8_t calc_checksum = countBits(0b1111111110000011 & sequence);
    uint8_t recv_checksum = (sequence & 0b0000000001111100) >> 2;

    uint8_t culprit_id = (sequence & 0b0111111110000000) >> 7;

    if (calc_checksum == recv_checksum) {
      infrared::got_blasted(culprit_id);
    } else {
      Serial.println("CS dont match");
    }
  }
}

void infrared::spin_rx() { analyzeBuffer(changes_1, &changes_1_size); }

void infrared::blast(uint8_t gunner_player_id) {
  uint16_t ircode = infrared::generate_ir_code(gunner_player_id);

  rmt_write_items(rmt_cfg.channel, ircode_2_rmt(ircode), 16, false);
}
