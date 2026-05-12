#ifdef ENV_ALPHA4
#include "cs1237.h"
#include "cs1237_app.h"
#include "sysclock.h"
#include "../../core/serial.h"

#include <library/inc/hc32f460_timer6.h>
#include <library/inc/hc32f460_interrupts.h>
#include <library/inc/hc32f460_pwc.h>
#include <library/inc/hc32f460_gpio.h>
#include "dwt.h"



struct cs1237_dev cs1237;

stc_port_init_t stcPortInit;

void st_timer_init(void)
{
  dwt_init();
}

int cs1237_delay_us(uint32_t us) {

    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us;
    
    while ((DWT->CYCCNT - start) < cycles);
    return 0;
}


int cs1237_gpio_init(void) {

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enLatch = Disable;
    stcPortInit.enExInt = Disable;
    stcPortInit.enInvert = Disable;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enPinDrv = Pin_Drv_H;
    stcPortInit.enPinOType = Pin_OType_Od;
    stcPortInit.enPinSubFunc = Disable;

    PORT_Init(CS1237_DRDY_PORT, CS1237_DRDY_PIN, &stcPortInit);
    PORT_Init(CS1237_SCK_PORT, CS1237_SCK_PIN, &stcPortInit);

    return 0;
}

int cs1237_drdy_mode_set(uint8_t mode) {

    if(mode == CS1237_DOUT_OUTPUT) {
        stcPortInit.enPinMode = Pin_Mode_Out;
        PORT_Init(CS1237_DRDY_PORT, CS1237_DRDY_PIN, &stcPortInit);
    }else {
        stcPortInit.enPinMode = Pin_Mode_In;
        PORT_Init(CS1237_DRDY_PORT, CS1237_DRDY_PIN, &stcPortInit);
    }    
    return 0;
}

int cs1237_drdy_write(uint8_t state) {

    if(state){
        PORT_SetBits(CS1237_DRDY_PORT, CS1237_DRDY_PIN);
    }
    else{
        PORT_ResetBits(CS1237_DRDY_PORT, CS1237_DRDY_PIN);
    }
        
    return 0;
}

uint8_t cs1237_drdy_read(void) {

    return (uint8_t)PORT_GetBit(CS1237_DRDY_PORT, CS1237_DRDY_PIN);
}


int cs1237_sck_write(uint8_t state) {

    if(state)
        PORT_SetBits(CS1237_SCK_PORT, CS1237_SCK_PIN);
    else
        PORT_ResetBits(CS1237_SCK_PORT, CS1237_SCK_PIN);

    return 0;
}

void cs1237_func_init(void) {
    cs1237.cs1237_gpio_init = cs1237_gpio_init;
    cs1237.cs1237_drdy_mode_set = cs1237_drdy_mode_set;
    cs1237.cs1237_drdy_write = cs1237_drdy_write;
    cs1237.cs1237_drdy_read = cs1237_drdy_read;
    cs1237.cs1237_sck_write = cs1237_sck_write;
    cs1237.cs1237_delay_us = cs1237_delay_us;

    cs1237.cs1237_gpio_init();

    st_timer_init();

    cs1237_config_init();

}

int32_t cs1237_data_dir(int32_t input_data) 
{
    int32_t output_data;

    if(input_data & (1 << 23)) {
        output_data = ~input_data;
        output_data = -((output_data + 1) & 0x00FFFFE0);
    }else {
        output_data = input_data;
        output_data = (output_data) & 0x0FFFFFE0;
    }

    return output_data;
}
float cs1237_data_deal(int32_t input_data) 
{
    int32_t output_data;
    float output_data_f;

    output_data = cs1237_data_dir(input_data);

    output_data_f = (output_data * ((0.5 * CS1237_VREF) / CS1237_GAIN )) / CS1237_ADC_BIT; //v

    return output_data_f;
}

// 执行清零的动作 (Perform the zeroing action)
void cs1237_set_zero(struct cs1237_dev *cs1237) {

    // 清零 (Zero out)
    int32_t cs1237_read_zero=0;
    int32_t temp;
    int32_t data_buf[ZERO_TIMES];
    
    cs1237_read_zero=0;
    
    for (uint8_t i=0; i<ZERO_TIMES;i++) {
        temp = cs1237_data_read(cs1237);
        while( temp == 0) {
            temp = cs1237_data_read(cs1237);
        }
        data_buf[i] = temp;
    }

    for (uint8_t j=0; j<ZERO_TIMES;j++) {

        cs1237_read_zero += data_buf[j] / ZERO_TIMES;
    }
    
    while(cs1237_read_zero == 0) // 正常数值不应该为0 (Normal value should not be 0)
    {
       cs1237_read_zero =  cs1237_data_read(cs1237);
    }

    cs1237->cs_zero_val = (int32_t)(cs1237_data_deal(cs1237_read_zero) * 1000000);  // μV  微伏 (microvolts)
}

void cs1237_set_threshold(int32_t thr)
{
    cs1237.cs_throshold = thr;
}

int32_t cs1237_get_threshold()
{
    return cs1237.cs_throshold;
}

int32_t cs1237_get_current_value()
{
    cs1237.cs_data = (int32_t)(cs1237_data_read(&cs1237));
    cs1237.cs_deal_val = (int32_t)(cs1237_data_deal(cs1237.cs_data)*1000000); // μV   微伏 (microvolts)

    return cs1237.cs_deal_val;
}

void cs1237_config_init()
{
  uint8_t cs1237_config = 0;

  cs1237_power_down(&cs1237);
  dwt_delay_ms(100);
  cs1237_write_config(&cs1237);
  dwt_delay_ms(100);  

  cs1237_write_config(&cs1237);
  cs1237_config = cs1237_read_config(&cs1237);

  if(cs1237_config != 0x3C)
  {
    cs1237_write_config(&cs1237);
  }
 
  cs1237.cs_trigger_state = 1;
  cs1237_set_threshold(CS1237_THRESHOLD);
  cs1237_set_zero(&cs1237);
}

//获取触发状态 (Get trigger state)
//根据configuration.h设置的Z_MIN_PROBE_ENDSTOP_HIT_STATE (Based on Z_MIN_PROBE_ENDSTOP_HIT_STATE set in configuration.h)
//如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为HIGH时，返回1为触发，发回0为未触发； (When HIGH: returns 1 = triggered, 0 = not triggered)
//如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为LOW时，返回0为触发，发回1为未触发； (When LOW: returns 0 = triggered, 1 = not triggered)
uint8_t cs1237_trigger()
{
    return cs1237.cs_trigger_state;
}

//根据触发阈值，计算当前压力值是否达到触发条件。 (Compare current pressure value against trigger threshold)
//此函数会根据阈值，计算得到cs1237.cs_trigger_state的状态值 (This function updates cs1237.cs_trigger_state based on the threshold)
//此函数在idle()调用。 (Called from idle())
void calc_cs1237_trigger_state()
{
    cs1237.cs_data = (int32_t)(cs1237_data_read(&cs1237));
    cs1237.cs_deal_val = (int32_t)(cs1237_data_deal(cs1237.cs_data)*1000000); //  μV   微伏 (microvolts)

    if(fabs(cs1237.cs_deal_val) <20) return;

    #if 0
    if(cs1237.last_data != 0) {
        cs1237.cs_deal_val = cs1237.cs_deal_val * CS1237_FIL + cs1237.last_data * (1 - CS1237_FIL);
    }
    cs1237.last_data = cs1237.cs_deal_val;

    #endif

    cs1237.cs_current_val = fabs((cs1237.cs_deal_val - cs1237.cs_zero_val)); 
        
    if(cs1237.cs_current_val >= cs1237.cs_throshold) {
        #if Z_MIN_PROBE_ENDSTOP_HIT_STATE == LOW
            cs1237.cs_trigger_state = 0;   // 相当于低电平触发 (Equivalent to active-low trigger)
        #else
            cs1237.cs_trigger_state = 1;   // 相当于高电平触发 (Equivalent to active-high trigger)
        #endif
    }else {
        #if Z_MIN_PROBE_ENDSTOP_HIT_STATE == LOW
            cs1237.cs_trigger_state = 1;   // 相当于低电平未触发 (Equivalent to active-low not triggered)
        #else
            cs1237.cs_trigger_state = 0;   // 相当于高电平未触发 (Equivalent to active-high not triggered)
        #endif
    }
       
}
#endif // ENV_ALPHA4
