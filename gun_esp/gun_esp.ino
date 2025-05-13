#include <LiquidCrystal_I2C.h>
#include <DFRobotDFPlayerMini.h>  // has to be 1.0.5, newer will get stuck on play()!!!
#include <SoftwareSerial.h>
#include <painlessMesh.h>
#include <Arduino.h>
#include <driver/rmt.h>
#include <Button2.h>
#include <ArduinoJson.h>


typedef struct {
  uint8_t id;
  uint8_t teamid;
  uint8_t hits = 0;
} Player;


enum Commands {
  CMD_PING,
  CMD_ACK,
  CMD_CHANGE_GR,
  CMD_START_TIMER,
  CMD_SEND_PLAYER_DATA,
  CMD_REQUEST_DATA,
  CMD_GAME_OVER,
  CMD_PLAYER_DATA_FINISHED,
  CMD_RESPAWN
};

enum IDs {
  ID_PC,
  ID_ALL,
  ID_ALL_PLAYERS
};

enum Gamerules {
  GR_LCD_Backlight,
  GR_Deadtime_s,
  GR_Friendly_Fire,
  GR_DoGunSound,
  GR_DoPointerLED,
  GR_maxDeaths,
  GR_maxShots,
  GR_limitShots,
  GR_RespawnMode,
  GR_StandardDamage,
  GR_StandardResistance,
  GR_DoRegeneration,
  GR_RegenerationSpeed
};


#define Audio_RX_Pin GPIO_NUM_39
#define Audio_TX_Pin GPIO_NUM_38

#define LeftButtonPin GPIO_NUM_21
#define RightButtonPin GPIO_NUM_48
#define OKButtonPin GPIO_NUM_47

#define IRLEDPin GPIO_NUM_42
#define BLUELEDPin GPIO_NUM_41
#define AUXLEDPin GPIO_NUM_40

#define SensPin1 GPIO_NUM_6


#define SOUND_NO_MORE_SHOTS 2
#define SOUND_PEW 3


#define   MESH_PREFIX     "LasertagMesh"
#define   MESH_PASSWORD   "00000000"


Player players[128];
uint8_t MY_ID;


Scheduler userScheduler;
painlessMesh mesh;
LiquidCrystal_I2C lcd(0x27, 16, 2);  
DFRobotDFPlayerMini audio;
SoftwareSerial AudioSerial(Audio_RX_Pin, Audio_TX_Pin);


void check_all_ir_buffers();
Task taskCheckIRBuffer(TASK_MILLISECOND*1, TASK_FOREVER, &check_all_ir_buffers);
rmt_config_t rmt_cfg;
static const rmt_item32_t rmt_item_high = {{{299, 1, 1, 0}}};
static const rmt_item32_t rmt_item_low = {{{299, 0, 1, 0}}};
void isr_sens_1();

Button2 OKButton;
Button2 LeftButton;
Button2 RightButton;
void _on_OKButton_pressed(Button2& b);
void _on_LeftButton_pressed(Button2& b);
void _on_RightButton_pressed(Button2& b);
void _on_OKButton_released(Button2& b);
void _on_LeftButton_released(Button2& b);
void _on_RightButton_released(Button2& b);


uint8_t getNodeId() {
  uint64_t mac = ESP.getEfuseMac();  // 48-bit MAC-Adresse
  // XOR-Faltung auf 8 Bit
  uint8_t id = (mac ^ (mac >> 8) ^ (mac >> 16) ^ (mac >> 24) ^ (mac >> 32) ^ (mac >> 40)) & 0xFF;
  return id;
}


void setup() {
  MY_ID = getNodeId();

  pinMode(BLUELEDPin, OUTPUT);
  pinMode(AUXLEDPin, OUTPUT);
  pinMode(IRLEDPin, OUTPUT);

  pinMode(SensPin1, INPUT_PULLUP);

  Serial.begin(9600);

  
  lcd.init();               
  lcd.backlight();
  lcd.print("LCD initialized.");

  AudioSerial.begin(9600);

  for(int i=0; i<10; i++) {

    if(audio.begin(AudioSerial, true, true)) {
      Serial.println("DFPlayer Mini online.");
      lcd.clear();
      lcd.print("Audio Online");

      audio.volume(15);
      audio.play(SOUND_PEW);
      break;
    }
    else {
      Serial.println("Audio Init Error.");
      lcd.clear();
      lcd.print("Audio Init Error");

      delay(500);
    }
  }

  mesh.setDebugMsgTypes( ERROR | STARTUP );
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  Serial.println("Mesh Done");

  lcd.clear();
  lcd.print("RMT IR...");

  //configure rmt and install driver
  rmt_cfg.channel = RMT_CHANNEL_0;
  rmt_cfg.rmt_mode = RMT_MODE_TX;
  rmt_cfg.gpio_num = IRLEDPin;
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


  //setup interrupts and atsk for IR receiver
  attachInterrupt(SensPin1, isr_sens_1, CHANGE);
  userScheduler.addTask(taskCheckIRBuffer);
  taskCheckIRBuffer.enable();

  // initialize buttons
  OKButton.begin(OKButtonPin);
  OKButton.setPressedHandler(_on_OKButton_pressed);
  OKButton.setReleasedHandler(_on_OKButton_released);

  LeftButton.begin(LeftButtonPin);
  LeftButton.setPressedHandler(_on_LeftButton_pressed);
  LeftButton.setReleasedHandler(_on_LeftButton_released);

  RightButton.begin(RightButtonPin);
  RightButton.setPressedHandler(_on_RightButton_pressed);
  RightButton.setReleasedHandler(_on_RightButton_released);

  lcd.clear();
  lcd.print("Waiting for timer command");
}


unsigned long T_lastShot = 0;



// MESH CODE


void RFsend(uint8_t id, uint8_t cmd, uint8_t arg1 = 0, uint8_t arg2 = 0, uint8_t arg3 = 0, uint8_t arg4 = 0, uint8_t arg5 = 0) {
  
  JsonDocument msg;
  msg["id"] = id;
  msg["cmd"] = cmd;
  msg["arg1"] = arg1;
  msg["arg2"] = arg2;
  msg["arg3"] = arg3;
  msg["arg4"] = arg4;
  msg["arg5"] = arg5;
  
  String JsonString;
  serializeJson(msg, JsonString);

  mesh.sendBroadcast(JsonString);
}


void receivedCallback( uint32_t from, String &json ) {
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

  if(!(id == MY_ID || id == ALL || id == ALL_PLAYERS)) {return;} // not meant

  switch(cmd) {

    case CMD_PING:
      RFsend(ID_PC, CMD_ACK);
      break;
    
    case CMD_START_TIMER:

      for(int i=arg1, i>0; i--) {
        lcd.clear();
        lcd.print(i);
        delay(1000);
      }
      block_init = false;
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}



// IR CODE



uint16_t generate_ir_code() {
  // returns message for shooting.
  // P: playerid, C: checksum
  // structure: 0b 1PPP PPPP PCCC CC01 (starts with 1 and ends with 01)

  // first, make frame:
  uint16_t ircode = 0b1000000000000001;

  // append playerid with space for 5 bits cs and 2 bits ending
  ircode += MY_ID << 7;

  // generate checksum
  uint8_t checksum = 0;
  for(int i=0; i<16; i++) {
    
    if(bitRead(ircode, i)) {checksum++;}
  }

  // write checksum in ircode
  ircode += checksum << 2;

  Serial.print("Ir Code Generated: ");
  Serial.println(ircode, BIN);

  return ircode;
}


void shoot() {

  T_lastShot = millis();

  digitalWrite(BLUELEDPin, HIGH);
  
  uint16_t ircode = generate_ir_code();


  // turn uint16 to array of rmt items.
  rmt_item32_t items[16];

  for(int i=0; i<16; i++) {

    if(bitRead(ircode, i)) {items[15-i] = rmt_item_high;} 
    else {items[15-i] = rmt_item_low;}
  }
  rmt_write_items(rmt_cfg.channel, items, 16, false);
  Serial.println("Sent.");

  digitalWrite(BLUELEDPin, LOW);
}


uint32_t changes_1[32] = {};
size_t changes_1_size = 0;


// log changes of sens pin
void IRAM_ATTR isr_sens_1() {
  changes_1[changes_1_size] = micros();
  changes_1_size++;
}


uint8_t countBits(uint16_t n) {
    uint8_t count = 0;
    while (n) {
        n &= (n - 1);  // delete least-significant bit
        count++;
    }
    return count;
}


// analyses IR buffer and determines, whether i was shot
void analyzeBuffer(uint32_t* buffer, size_t* buffer_size) {

  // if buffer is not empty and first entry is older than 10ms, analyse
  if(buffer[0] && micros() - buffer[0] >= 10000) {

    // 1st: check for time diff under 150us -> error
    for(size_t i = 1; i < *buffer_size; i++) {
      if(buffer[i] - buffer[i - 1] < 150) {
        Serial.println("Too tight timing.");
        return;
      }
    }
    Serial.println("timing ok.");

    // generate binary sequence from timestamps
    uint16_t sequence;
    uint8_t bit = 0;
    bool state = true;

    for(size_t i = 1; i < *buffer_size; i++) {  // read out data
      int diff = buffer[i] - buffer[i - 1];
      int nearest_multiple = round(diff / 300.0);  // find nearest multiple to 300usec -> append that many digits
      
      for(int j = 0; j < nearest_multiple; j++) {  // append
        if (bit < 16) {  // prevent overflow

          bitWrite(sequence, 15-bit, state);
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

    if(calc_checksum == recv_checksum) {
      Serial.println("Hit!");
    }
    else {
      Serial.println("CS dont match");
    }

  }
}


void check_all_ir_buffers() {

  analyzeBuffer(changes_1, &changes_1_size);
}



void loop() {
  mesh.update();
  OKButton.loop();
  LeftButton.loop();
  RightButton.loop();
}


// BUTTON CODE STARTS HERE


// OK
void _on_OKButton_pressed(Button2& b) {
  Serial.println("OK pressed");

  if(millis() > T_lastShot+1000) {
    Serial.println("Pew!");
    shoot();
    lcd.clear();
    lcd.print("ok");
  }
}

void _on_OKButton_released(Button2& b) {
  Serial.println("Ok released");
}


// LEFT
void _on_LeftButton_pressed(Button2& b) {
  Serial.println("Left pressed");
  lcd.clear();
  lcd.print("left");
  digitalWrite(BLUELEDPin, HIGH);
}

void _on_LeftButton_released(Button2& b) {
  Serial.println("Left released");
  digitalWrite(BLUELEDPin, LOW);
}


// RIGHT
void _on_RightButton_pressed(Button2& b) {
  RFsend(ALL, PING);
  Serial.println("Right pressed");
}

void _on_RightButton_released(Button2& b) {
  Serial.println("Right released");
}



