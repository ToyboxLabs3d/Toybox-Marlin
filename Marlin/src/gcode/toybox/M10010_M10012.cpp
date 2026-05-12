#ifdef ENV_ALPHA4

#include "../gcode.h"
#include  "../../HAL/HC32/cs1237_app.h"
#include  "../../HAL/HC32/cs1237.h"
//e.g:"M10010 S100 \n"     set threshold 0.1mV
void GcodeSuite::M10010()  //set threshold  
{
    if (parser.seenval('S')) {
        const int16_t x = parser.value_int();
        cs1237_set_threshold((int32_t)x);
    }
}
//e.g: "M10011\n"
void GcodeSuite::M10011()
{
    SERIAL_ECHOLNPGM("cs1237 current value: ", cs1237_get_current_value()); 
}

void GcodeSuite::M10012()
{

   SERIAL_ECHOLNPGM("cs1237 zero value: ",cs1237.cs_zero_val);
}
#endif