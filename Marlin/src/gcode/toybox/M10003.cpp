
#if HAS_FILAMENT_SENSOR

#include "../gcode.h"
#include "../queue.h"
#include "../../MarlinCore.h"

void GcodeSuite::M10003() {
    // Alert the ESP32 that filament is out
    SERIAL_ECHO_MSG("filament_out");
    millis_t start = millis();
    // Give ESP32 a chance to stop sending commands.
    while(millis() - start < 1000) {
        idle();
    }
    do{
        SERIAL_IMPL.flush();
        for(auto i = 0; i < NUM_SERIAL; i++){
            while (SERIAL_IMPL.read(i) != -1) {
                // Just ignore
            }
        }
        queue.clear(); // empty the queue
        queue.get_available_commands(); // refill the queue from serial
    } while (queue.has_commands_queued());
    SERIAL_ECHO_MSG("queue cleared");
}

#endif