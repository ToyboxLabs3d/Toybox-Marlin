
#include "fast_commands.h"
#include "../../gcode/queue.h"
#include "../../MarlinCore.h"
#include "../../module/temperature.h"
#include "../../module/planner.h"

#if ENABLED(TOYBOX_FAST_CMDS) || HAS_FILAMENT_SENSOR
static void clear_queue_and_serial(){
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
}
#endif

#if ENABLED(TOYBOX_FAST_CMDS)

void fast_cancel(){
    SERIAL_ECHOLNPGM(">> Fast Cancel");
    clear_queue_and_serial();
    
    stop_running_move = true;
    thermalManager.disable_all_heaters();

    SERIAL_ECHOLNPGM("fast-cancel-done");
}

void fast_pause(){
    SERIAL_ECHOLNPGM(">> Fast Pause");
    planner.need_to_clear = true;
    clear_queue_and_serial();
    planner.synchronize();
    planner.process_cleared_lines();
    SERIAL_ECHO_MSG("synced-after-stop");
}


#endif // TOYBOX_FAST_CMDS

#if HAS_FILAMENT_SENSOR
void on_filament_runout(){
    SERIAL_ECHO_MSG("filament_runout");
    // stop_after_current_line();
    planner.need_to_clear = true;

    millis_t start = millis();
    // Give ESP32 a chance to stop sending commands.
    while(millis() - start < 1000) {
        idle();
    }
    clear_queue_and_serial();

    planner.synchronize();
    planner.process_cleared_lines();
    // SERIAL_ECHO_MSG("synchronized planner after runout event");
    SERIAL_ECHO_MSG("synced-after-stop");
}
#endif // HAS_FILAMENT_SENSOR