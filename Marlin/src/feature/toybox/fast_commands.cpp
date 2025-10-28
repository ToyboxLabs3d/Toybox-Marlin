#if ENABLED(TOYBOX_FAST_CMDS)

#include "fast_commands.h"
#include "../../gcode/queue.h"
#include "../../MarlinCore.h"
#include "../../module/temperature.h"

void fast_cancel(){
    SERIAL_ECHOLNPGM(">> Fast Cancel");
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
    
    stop_running_move = true;
    thermalManager.disable_all_heaters();

    SERIAL_ECHOLNPGM("fast-cancel-done");
}

void fast_pause(){
    SERIAL_ECHOLNPGM(">> Fast Pause");
    SERIAL_ECHOLNPGM("fast-pause-done");
}

#endif // TOYBOX_FAST_CMDS