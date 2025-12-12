#pragma once

#if ENABLED(TOYBOX_FAST_CMDS)

void fast_cancel();
void fast_pause();

#endif // TOYBOX_FAST_CMDS

#if HAS_FILAMENT_SENSOR
void on_filament_runout();
#endif // HAS_FILAMENT_SENSOR