#include "buttons.h"
#include "config.h"
#include "funk.h"
#include "infrared.h"
#include "sound.h"
#include <LiquidCrystal_I2C.h>

enum Gamerules {};

enum STATES {
  initializing,
  waiting_for_timer,
  countdown,
  active_game,
  game_ended
};
STATES state = initializing;

uint8_t MY_ID;
uint8_t TEAM = 0;
int Health = 500;
int deadtime_s = 5;
unsigned long T_lastShot = 0;

Scheduler userScheduler;
Task task_ir_buffer(TASK_MILLISECOND * 1, TASK_FOREVER, &infrared::spin_rx);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void start_game();
void die();
void shoot();
uint8_t getNodeId();

void setup() {
  MY_ID = getNodeId();

  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_AUX, OUTPUT);
  pinMode(PIN_LED_IR, OUTPUT);

  pinMode(PIN_IR_RX, INPUT_PULLUP);

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.print("LCD initialized.");

  buttons::init();
  Serial.println("Buttons Init");
  sound::setup();
  Serial.println("sound Init");
  sound::play(SOUND_STARTUP);
  infrared::init();
  Serial.println("Infrared Init");
  funk::begin(&userScheduler);
  Serial.println("Mesh Init");

  lcd.clear();
  lcd.print("Systems Online");

  buttons::get_button(buttons::OK).setClickHandler([&](Button2 btn) {
    Serial.print("OK clicked");
    // TODO u sure?
    lcd.clear();
    lcd.print("ok");
  });

  buttons::get_button(buttons::LEFT).setChangedHandler([&](Button2 btn) {
    if (btn.isPressed()) {
      Serial.print("LEFT pressed");
      lcd.clear();
      lcd.print("left");
      digitalWrite(PIN_LED_BLUE, HIGH);
    } else {
      Serial.print("LEFT released");

      digitalWrite(PIN_LED_BLUE, LOW);
    }
  });

  buttons::get_button(buttons::RIGHT).setClickHandler([&](Button2 btn) {
    Serial.print("RIGHT clicked");
    funk::send(ID_ALL, CMD_PING);
  });

  buttons::get_button(buttons::TRIGGER).setPressedHandler([&](Button2 btn) {
    Serial.print("TRIGGER pressed");
    if (millis() > T_lastShot + 1000) {
      Serial.println("Pew!");
      shoot();
    }
  });

  userScheduler.addTask(task_ir_buffer);
  task_ir_buffer.enable();

  state = waiting_for_timer;
  lcd.clear();
  lcd.print("Waiting for timer command");

  while (state == waiting_for_timer) {
    funk::spin();
    buttons::spin_only(buttons::OK);

    if (buttons::get_button(buttons::OK).isPressed()) {
      lcd.clear();
      lcd.print("skipped");
      start_game();
      break;
    }
  }
  Serial.println("Exited setup function");
}

uint8_t getNodeId() {
  uint64_t mac = ESP.getEfuseMac(); // 48-bit MAC-Adresse
  // XOR-Faltung auf 8 Bit
  uint8_t id = (mac ^ (mac >> 8) ^ (mac >> 16) ^ (mac >> 24) ^ (mac >> 32) ^
                (mac >> 40)) &
               0xFF;
  return id;
}

void update_health_bar() {

  if (Health <= 0) {
    die();
  }

  // update health blocks
  lcd.rightToLeft();
  lcd.setCursor(15, 0);
  lcd.print("        ");
  lcd.setCursor(15, 0);

  int blocks = (8 * Health) / 500;

  for (int i = 0; i < blocks; i++) {
    lcd.write(0xFF);
  }
  lcd.leftToRight();
}

void start_game() {

  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print("#");
  lcd.print(MY_ID, HEX);

  lcd.setCursor(9, 1);
  lcd.print("TEAM ");
  lcd.print(TEAM, HEX);

  lcd.setCursor(0, 0);
  lcd.print("Alive");

  state = active_game;

  update_health_bar();
}

void funk::rx_callback(uint32_t from, String &json) {
  Serial.printf("Received from %u msg=%s\n", from, json.c_str());

  JsonDocument msg;
  deserializeJson(msg, json);

  uint8_t id = msg["id"].as<uint8_t>();
  uint8_t cmd = msg["cmd"].as<uint8_t>();
  uint8_t arg1 = msg["arg1"].as<uint8_t>();
  uint8_t arg2 = msg["arg2"].as<uint8_t>();
  uint8_t arg3 = msg["arg3"].as<uint8_t>();
  uint8_t arg4 = msg["arg4"].as<uint8_t>();
  uint8_t arg5 = msg["arg5"].as<uint8_t>();

  if (!(id == MY_ID || id == ID_ALL || id == ID_ALL_PLAYERS)) {
    return;
  } // not meant

  switch (cmd) {

  case CMD_PING:
    funk::send(ID_PC, CMD_ACK);
    break;

  case CMD_START_TIMER:

    for (int i = arg1; i > 0; i--) {
      lcd.clear();
      lcd.print(i);
      delay(1000);
    }
    start_game();
    break;

  case CMD_END_GAME:
    state = game_ended;
    lcd.clear();
    lcd.print("Game Over!");
  }
}

void shoot() {
  T_lastShot = millis();
  sound::play(SOUND_PEW);

  digitalWrite(PIN_LED_BLUE, HIGH);

  infrared::blast(MY_ID);
  Serial.println("Sent.");

  delay(500);

  digitalWrite(PIN_LED_BLUE, LOW);
}

void infrared::got_blasted(uint8_t from_player_id) {
  Serial.println("Hit!");
  Health -= 100;
  update_health_bar();
  funk::send(ID_PC, CMD_HIT_EVENT, from_player_id, MY_ID);
}

uint32_t T_died = 0;
void die() {
  Health = 0;

  lcd.setCursor(0, 0);
  lcd.print("Dead");
  lcd.noBacklight();

  T_died = millis();
}

void respawn() {

  Health = 500;

  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print("#");
  lcd.print(MY_ID, HEX);

  lcd.setCursor(9, 1);
  lcd.print("TEAM ");
  lcd.print(TEAM, HEX);

  lcd.setCursor(0, 0);
  lcd.print("Alive");

  update_health_bar();
}

void loop() {
  funk::spin();
  buttons::spin();

  if (Health <= 0 && millis() - T_died > deadtime_s * 1000) {
    respawn();
  }
}
