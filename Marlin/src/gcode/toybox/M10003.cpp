
#ifdef ENV_CHARLIE

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
    queue.clear();
    SERIAL_ECHO_MSG("queue cleared");
}

#endif