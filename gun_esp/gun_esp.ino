#include <LiquidCrystal_I2C.h>
#include <DFRobotDFPlayerMini.h>  // has to be 1.0.5, newer will get stuck on play()!!!
#include <SoftwareSerial.h>
#include <painlessMesh.h>
#include <Arduino.h>
#include <driver/rmt.h>


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


const uint8_t MY_ID = 13;

bool limit_shots = false;
bool do_gun_sound = true;
int TOT_SHOTS = 0;
int max_shots = 9999;
bool do_pointer_led = true;


Scheduler userScheduler;
painlessMesh mesh;
LiquidCrystal_I2C lcd(0x27, 16, 2);  
DFRobotDFPlayerMini audio;
SoftwareSerial AudioSerial(Audio_RX_Pin, Audio_TX_Pin);


void sendMessage();
void check_all_ir_buffers();
Task taskSendMessage(TASK_SECOND * 1, TASK_FOREVER, &sendMessage);
Task taskCheckIRBuffer(TASK_MILLISECOND*1, TASK_FOREVER, &check_all_ir_buffers);
rmt_config_t rmt_cfg;
static const rmt_item32_t rmt_item_high = {{{299, 1, 1, 0}}};
static const rmt_item32_t rmt_item_low = {{{299, 0, 1, 0}}};
void isr_sens_1();


void setup() {
  pinMode(LeftButtonPin, INPUT_PULLUP);
  pinMode(RightButtonPin, INPUT_PULLUP);
  pinMode(OKButtonPin, INPUT_PULLUP);
  pinMode(BLUELEDPin, OUTPUT);
  pinMode(AUXLEDPin, OUTPUT);
  pinMode(IRLEDPin, OUTPUT);

  pinMode(SensPin1, INPUT_PULLUP);

  Serial.begin(9600);

  
  lcd.init();               
  lcd.backlight();
  lcd.print("LCD initialized.");

  AudioSerial.begin(9600);

  while (!audio.begin(AudioSerial, /*isACK = */ true, /*doReset = */ true)) {  //Use serial to communicate with mp3.
    Serial.println("Audio Init Error");
    lcd.clear();
    lcd.print("Audio Init Error");
  }
  Serial.println("DFPlayer Mini online.");
  lcd.clear();
  lcd.print("Audio Online");

  audio.volume(15);
  audio.play(SOUND_PEW);

  mesh.setDebugMsgTypes( ERROR | STARTUP );
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
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


  attachInterrupt(SensPin1, isr_sens_1, CHANGE);
  userScheduler.addTask(taskCheckIRBuffer);
  taskCheckIRBuffer.enable();
}


unsigned long T_lastShot = 0;



// MESH CODE

void sendMessage() {
  String msg = "Hi from node1";
  msg += mesh.getNodeId();
  mesh.sendBroadcast(msg);
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}


void RFsend(char id, char cmd, char arg1 = 0, char arg2 = 0, char arg3 = 0, char arg4 = 0, char arg5 = 0) {
  //mesh.sendBroadcast(id, cmd, arg1, arg2, arg3, arg4, arg5);
}


void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
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
  

  if(digitalRead(LeftButtonPin) == LOW) {
    Serial.println("Left");
    lcd.clear();
    lcd.print("left");
    digitalWrite(BLUELEDPin, HIGH);
  }
  else {
    digitalWrite(BLUELEDPin, LOW);
  }


  if(digitalRead(RightButtonPin) == LOW) {
    Serial.println("Right");
    lcd.clear();
    lcd.print("right");
  }
  else {
  }


  if(digitalRead(OKButtonPin) == LOW) {
    Serial.println("OK");

    if(millis() > T_lastShot+1000) {
      Serial.println("Pew!");
      shoot();
      lcd.clear();
      lcd.print("ok");
    }
  }
}
