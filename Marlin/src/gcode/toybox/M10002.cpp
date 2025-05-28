#include "../gcode.h"


#define BUILD_NUMBER 1
#define VERSION_STRING "1.0.0"

void GcodeSuite::M10002(){
  SERIAL_ECHO_MSG("Toybox-marlin BUILD: ", BUILD_NUMBER, " Version: ", VERSION_STRING);
//   SERIAL_ECHO_MSG("Toybox-marlin BUILD");
}