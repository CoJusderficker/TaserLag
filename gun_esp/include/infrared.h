#include <driver/rmt.h>
#include <task.h>
#include <Arduino.h>


namespace infrared {
    void init();

    void spin_rx();

    uint16_t generate_ir_code(uint8_t player_id);

    void blast(uint8_t gunner_player_id);
    void got_blasted(uint8_t from_player_id);


} // namespace infrared
