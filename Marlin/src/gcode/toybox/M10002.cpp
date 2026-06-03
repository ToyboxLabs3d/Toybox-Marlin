#include "../gcode.h"

#if defined(ENV_ALPHA3) 
    #define BUILD_NUMBER 10
    #define VERSION_STRING "1.0.1"
#elif defined(ENV_ALPHA4)
    #define BUILD_NUMBER 208
    #define VERSION_STRING "1.0.0"
#elif defined(ENV_CHARLIE)
    #define BUILD_NUMBER 101
    #define VERSION_STRING "1.0.0"
#else
    #define BUILD_NUMBER (-1)
    #define VERSION_STRING "Unknown"
#endif

void GcodeSuite::M10002(){
  SERIAL_ECHO_MSG("Toybox-marlin BUILD: " STRINGIFY(BUILD_NUMBER) " VERSION: " VERSION_STRING);
}