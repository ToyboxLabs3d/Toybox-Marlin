#ifdef TOYBOX_PROBE_FUDGING

#include "../gcode.h"
#include "../../module/probe.h"

void GcodeSuite::M10013()  //set bed probing z fudge-factor
{
    if (parser.seenval('Z')) {
        const float z_offset = parser.value_linear_units();
        Probe::set_z_offset_fudge_factor(z_offset);
    } else {
        SERIAL_ECHOLNPGM("Probing Z offset fudge factor: ", Probe::get_z_offset_fudge_factor());
    }
}

#endif