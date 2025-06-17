#include "../gcode.h"

#ifdef ENV_ALPHA3
    #define BUILD_NUMBER 3
    #define VERSION_STRING "1.0.0"
#elif defined(ENV_CHARLIE)
    #define BUILD_NUMBER 100
    #define VERSION_STRING "1.0.0"
#else
    #define BUILD_NUMBER (-1)
    #define VERSION_STRING "Unknown"
#endif

void GcodeSuite::M10002(){
  SERIAL_ECHO_MSG("Toybox-marlin BUILD: ", BUILD_NUMBER, " VERSION: ", VERSION_STRING);
}