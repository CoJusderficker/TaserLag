#include <painlessMesh.h>
#include <Arduino.h>
#include <ArduinoJson.h>


typedef struct {
  uint8_t id;
  uint8_t teamid;
  uint8_t hits = 0;
} Player;


enum Commands {
  CMD_PING,
  CMD_ACK,
  CMD_START_TIMER,
  CMD_HIT_EVENT,
  CMD_END_GAME
};

enum IDs {
  ID_PC,
  ID_ALL,
  ID_ALL_PLAYERS
};

enum Gamerules {
};


#define   MESH_PREFIX     "LasertagMesh"
#define   MESH_PASSWORD   "00000000"


Player players[128];
uint8_t MY_ID = 1; // im the pc


Scheduler userScheduler;
painlessMesh mesh;


void setup() {
  
  Serial.begin(115200);
  Serial.println("Started");

  mesh.setDebugMsgTypes( ERROR | STARTUP );
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}


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

  if(!(id == ID_PC || id == ID_ALL)) {return;} // not meant
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.println("List of Mesh nodes:");

  Serial.println(mesh.getNodeList().front());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}



void loop() {
  mesh.update();

  if(Serial.available()) {
    char data = Serial.read();

    switch(data) {

    case 'a': {
      RFsend(ID_ALL, CMD_START_TIMER, 10);
      Serial.println("Sent Start Command");
      break;
    }
    
    case 'b': {
      std::list<uint32_t>members = mesh.getNodeList();
      for(uint32_t node : members) {
        Serial.println(node);
      }
      break;
    }
    
    case 'c': {
      RFsend(ID_ALL, CMD_END_GAME);
      Serial.println("game ended command sent")
      break;
    }

    }
  }
}
