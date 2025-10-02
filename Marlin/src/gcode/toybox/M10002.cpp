#include "../gcode.h"

#ifdef ENV_ALPHA3
    #define BUILD_NUMBER 6
    #define VERSION_STRING "1.0.1"
#elif defined(ENV_CHARLIE)
    #define BUILD_NUMBER 101
    #define VERSION_STRING "1.0.0"
#else
    #define BUILD_NUMBER (-1)
    #define VERSION_STRING "Unknown"
#endif

void GcodeSuite::M10002(){
  SERIAL_ECHO_MSG("Toybox-marlin BUILD: ", BUILD_NUMBER, " VERSION: ", VERSION_STRING);
}