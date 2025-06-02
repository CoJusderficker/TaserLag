#include <painlessMesh.h>
#include <ArduinoJson.h>



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

namespace funk
{
    void begin(Scheduler* scheduler);

    void spin();

    void rx_callback(uint32_t from, String &json);

    void send(uint8_t id, uint8_t cmd, uint8_t arg1 = 0, uint8_t arg2 = 0, uint8_t arg3 = 0, uint8_t arg4 = 0, uint8_t arg5 = 0);
} // namespace funk
