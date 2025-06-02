#include "funk.h"
#include "config.h"

painlessMesh mesh;

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() { Serial.printf("Changed connections\n"); }

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void funk::begin(Scheduler *scheduler) {
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, scheduler);
  mesh.onReceive(&funk::rx_callback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}

void funk::spin() { mesh.update(); }

// MESH CODE

void funk::send(uint8_t id, uint8_t cmd, uint8_t arg1, uint8_t arg2,
                uint8_t arg3, uint8_t arg4, uint8_t arg5) {

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
