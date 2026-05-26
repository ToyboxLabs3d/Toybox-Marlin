#ifdef ENV_ALPHA4

#include "cs1237.h"
#include "../../core/serial.h"
#include "sysclock.h"
#include "../../core/serial.h"
#include <library/inc/hc32f460_timer6.h>
#include <library/inc/hc32f460_interrupts.h>
#include <library/inc/hc32f460_pwc.h>
#include "dwt.h"

#define CS1237_MINI_PLUSE_US         1 //91 //40      
#define CS1237_RST_PLUSE_US         200//12000    

#define CS1237_DOUT_OUTPUT  0x0    
#define CS1237_DOUT_INTPUT  0x1

#define CS1237_SCK_PORT     PortA
#define CS1237_SCK_PIN      Pin12
#define CS1237_DRDY_PORT    PortA
#define CS1237_DRDY_PIN     Pin11

#define CS1237_VREF                     4.89
#define CS1237_GAIN                     128
#define CS1237_ADC_BIT                  8388607
#define CS1237_FIL                      0.9 

#define ZERO_TIMES                      1

#define NANOS_PER_SEC 1'000'000'000ULL
#define MICROS_PER_SEC 1'000'000ULL

#define NUM_PREV_VALUES 5

static_assert(NUM_PREV_VALUES % 2 == 1, "NUM_PREV_VALUES must be odd to have a clear median");

int32_t prev_values[NUM_PREV_VALUES] = {0};
int8_t prev_values_index = -1;


static void st_timer_init(void) {
    dwt_init();
}


static int delay_us(uint32_t us) {

    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = uint32_t(uint64_t(us) * F_SYSTEM_CLOCK / MICROS_PER_SEC);
    
    while ((DWT->CYCCNT - start) < cycles);
    return 0;
}

static int delay_ns(uint32_t ns) {

    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = uint32_t(uint64_t(ns) * F_SYSTEM_CLOCK / NANOS_PER_SEC);
    
    while ((DWT->CYCCNT - start) < cycles);
    return 0;
}


int CS1237::gpio_init() {

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

int CS1237::drdy_mode_set(uint8_t mode) {

    if(mode == CS1237_DOUT_OUTPUT) {
        stcPortInit.enPinMode = Pin_Mode_Out;
        PORT_Init(CS1237_DRDY_PORT, CS1237_DRDY_PIN, &stcPortInit);
    }else {
        stcPortInit.enPinMode = Pin_Mode_In;
        PORT_Init(CS1237_DRDY_PORT, CS1237_DRDY_PIN, &stcPortInit);
    }    
    return 0;
}

int CS1237::drdy_write(uint8_t state) {
    if(state){
        PORT_SetBits(CS1237_DRDY_PORT, CS1237_DRDY_PIN);
    }
    else{
        PORT_ResetBits(CS1237_DRDY_PORT, CS1237_DRDY_PIN);
    }   
    return 0;
}

uint8_t CS1237::drdy_read(void) {

    return (uint8_t)PORT_GetBit(CS1237_DRDY_PORT, CS1237_DRDY_PIN);
}


int CS1237::sck_write(uint8_t state) {

    if(state)
        PORT_SetBits(CS1237_SCK_PORT, CS1237_SCK_PIN);
    else
        PORT_ResetBits(CS1237_SCK_PORT, CS1237_SCK_PIN);

    return 0;
}

void CS1237::func_init(void) {
    gpio_init();
    st_timer_init();
    config_init();
}

int32_t CS1237::data_dir(int32_t input_data) 
{
    int32_t output_data;

    if(input_data & (1 << 23)) {
        output_data = ~input_data;
        output_data = -((output_data + 1) & 0x00FF'FFE0);
    }else {
        output_data = input_data;
        output_data = (output_data) & 0x0FFF'FFE0;
    }

    return output_data;
}
float CS1237::data_deal(int32_t input_data) 
{
    int32_t output_data;
    float output_data_f;

    output_data = data_dir(input_data);

    output_data_f = (output_data * ((0.5 * CS1237_VREF) / CS1237_GAIN )) / CS1237_ADC_BIT; //v

    return output_data_f;
}

// 执行清零的动作 (Perform the zeroing action)
void CS1237::set_zero() {

    // 清零 (Zero out)
    int32_t read_zero=0;
    int32_t temp;
    int32_t data_buf[ZERO_TIMES];
    
    read_zero=0;
    
    for (uint8_t i=0; i<ZERO_TIMES;i++) {
        temp = data_read();
        while( temp == 0) {
            temp = data_read();
        }
        data_buf[i] = temp;
    }

    for (uint8_t j=0; j<ZERO_TIMES;j++) {

        read_zero += data_buf[j] / ZERO_TIMES;
    }
    
    while(read_zero == 0) // 正常数值不应该为0 (Normal value should not be 0)
    {
       read_zero =  data_read();
    }

    cs_zero_val = (int32_t)(data_deal(read_zero) * 1000000);  // μV  微伏 (microvolts)
}

void CS1237::set_threshold(int32_t thr)
{
    cs_throshold = thr;
}

int32_t CS1237::get_threshold()
{
    return cs_throshold;
}

uint32_t CS1237::get_raw_data()
{
    return data_read();
}


static int32_t get_median_of_prev_values() {
    int32_t to_sort[NUM_PREV_VALUES];
    memcpy(to_sort, prev_values, sizeof(prev_values));

    const int median_index = NUM_PREV_VALUES / 2;

    // enough of an insertion sort to get the median value in place; no need to fully sort the array
    for(int sort_iter=0; sort_iter <= median_index; sort_iter++) {
        int32_t min_value = to_sort[sort_iter];
        int min_index = sort_iter;
        for(int canidate_idx=sort_iter+1; canidate_idx < NUM_PREV_VALUES; canidate_idx++) {
            if(to_sort[canidate_idx] < min_value) {
                min_value = to_sort[canidate_idx];
                min_index = canidate_idx;
            }
        }
        to_sort[min_index] = to_sort[sort_iter];
        to_sort[sort_iter] = min_value;
    }    

    return to_sort[median_index];
}

int32_t CS1237::get_current_value()
{

    if(prev_values_index == -1){
        for(prev_values_index = 0; prev_values_index < NUM_PREV_VALUES-1; prev_values_index++){
            prev_values[prev_values_index] = data_deal(data_read()) * 1000000; // μV   微伏 (microvolts)
            // SERIAL_ECHOLNPGM("Initializing CS1237 value buffer, reading value ", prev_values_index, "/", NUM_PREV_VALUES, "with ", prev_values[prev_values_index], " μV");
        }
    }

    cs_data = (int32_t)(data_read());
    cs_deal_val = (int32_t)(data_deal(cs_data)*1000000); // μV   微伏 (microvolts)
    prev_values[prev_values_index] = cs_deal_val;

    prev_values_index = (prev_values_index + 1) % NUM_PREV_VALUES;

    return get_median_of_prev_values();
}

void CS1237::config_init()
{
  uint8_t config = 0;

  power_down();
  dwt_delay_ms(100);
  write_config();
  dwt_delay_ms(100);  

  write_config();
  config = read_config();

  if(config != 0x3C)
  {
    write_config();
  }
 
  cs_trigger_state = 1;
  set_threshold(CS1237_THRESHOLD);
  set_zero();
}

//获取触发状态 (Get trigger state)
//根据configuration.h设置的Z_MIN_PROBE_ENDSTOP_HIT_STATE (Based on Z_MIN_PROBE_ENDSTOP_HIT_STATE set in configuration.h)
//如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为HIGH时，返回1为触发，发回0为未触发； (When HIGH: returns 1 = triggered, 0 = not triggered)
//如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为LOW时，返回0为触发，发回1为未触发； (When LOW: returns 0 = triggered, 1 = not triggered)
uint8_t CS1237::trigger()
{
    return cs_trigger_state;
}

//根据触发阈值，计算当前压力值是否达到触发条件。 (Compare current pressure value against trigger threshold)
//此函数会根据阈值，计算得到cs1237.cs_trigger_state的状态值 (This function updates cs1237.cs_trigger_state based on the threshold)
//此函数在idle()调用。 (Called from idle())
void CS1237::calc_trigger_state()
{
    cs_data = (int32_t)(data_read());
    cs_deal_val = (int32_t)(data_deal(cs_data)*1000000); //  μV   微伏 (microvolts)

    if(fabs(cs_deal_val) <20) {
        return;
    }

    cs_current_val = fabs((cs_deal_val - cs_zero_val)); 
        
    if(cs_current_val >= cs_throshold) {
        #if Z_MIN_PROBE_ENDSTOP_HIT_STATE == LOW
            cs_trigger_state = 0;   // 相当于低电平触发 (Equivalent to active-low trigger)
        #else
            cs_trigger_state = 1;   // 相当于高电平触发 (Equivalent to active-high trigger)
        #endif
    }else {
        #if Z_MIN_PROBE_ENDSTOP_HIT_STATE == LOW
            cs_trigger_state = 1;   // 相当于低电平未触发 (Equivalent to active-low not triggered)
        #else
            cs_trigger_state = 0;   // 相当于高电平未触发 (Equivalent to active-high not triggered)
        #endif
    }
       
}

/*********************************************************************************************************************
 *                                                 CS1237 驱动 (CS1237 driver)
*********************************************************************************************************************/



 void CS1237::_write_bit(uint8_t bit) {
    sck_write(1);
    delay_us(CS1237_MINI_PLUSE_US);
    drdy_write(bit);
    sck_write(0);
    delay_us(CS1237_MINI_PLUSE_US);
}

 void CS1237::_build_data(){
    drdy_mode_set(CS1237_DOUT_OUTPUT);
    drdy_write(1);
    drdy_mode_set(CS1237_DOUT_INTPUT);
    sck_write(0);
}

 bool CS1237::_wait_drdy_ready(uint32_t timeout_ms) {
    const uint32_t start_ms = millis();

    while (drdy_read() == 1) {
        if ((millis() - start_ms) >= timeout_ms) {
            SERIAL_ECHOLN("Error: CS1237 DRDY not ready within timeout (", timeout_ms, " ms)  delta ms: ", millis() - start_ms);
            return false; // Timeout
        }
    }
    return true; // Ready
}

/************************************************************************
 *  0x5c    //REF输出关闭，输出40hz     PGA=128(有效分辨率为20bit)  通道A  *  (REF off, 40Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x4c    //REF输出关闭，输出10hz     PGA=128(有效分辨率为20bit)  通道A  *  (REF off, 10Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x1c    //REF输出开启，输出40hz     PGA=128(有效分辨率为20bit)  通道A  *  (REF on, 40Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x2c    //REF输出开启，输出640hz    PGA=128(有效分辨率为20bit)  通道A  *  (REF on, 640Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x3c    //REF输出开启，输出1280hz   PGA=128(有效分辨率为20bit)  通道A  *  (REF on, 1280Hz output, PGA=128 [effective 20-bit], channel A)
*************************************************************************/
int CS1237::write_config() {

    uint8_t i;
    uint8_t data = 0x2c;

    _build_data();

    bool success = _wait_drdy_ready(1000); // 等待DRDY准备好，超时时间1000ms (Wait for DRDY to be ready, timeout 1000ms)
    if(!success) {
        SERIAL_ECHOLNPGM("Error: CS1237 DRDY not ready within timeout during config write");
        return -1; // DRDY not ready within timeout
    }

    //29个CLK脉冲 (29 CLK pulses)
    for (int i=0; i<29; i++) {
        sck_write(1);
        delay_us(CS1237_MINI_PLUSE_US);
        sck_write(0);
        delay_us(CS1237_MINI_PLUSE_US);
    }

    //第30~36个脉冲，写配置寄存器 (Pulses 30-36, write configuration register)
    drdy_mode_set(CS1237_DOUT_OUTPUT);
    _write_bit(1);
    _write_bit(1);
    _write_bit(0);
    _write_bit(0);
    _write_bit(1);
    _write_bit(0);
    _write_bit(1);

    sck_write(1);
    delay_us(CS1237_MINI_PLUSE_US);
    sck_write(0);
    delay_us(CS1237_MINI_PLUSE_US);

    //第38~45个脉冲，写８位数据 (Pulses 38-45, write 8 bits of data)
	for(i=0; i < 8; i++)
	{
		sck_write(1);
		delay_us(CS1237_MINI_PLUSE_US);

		if(data & 0x80)
			drdy_write(1);
		else
			drdy_write(0);
		
		data <<= 1;
		
		sck_write(0);
		delay_us(CS1237_MINI_PLUSE_US);	
	}

    //第46个脉冲结束，并释放引脚 (End of pulse 46; release the pin)
    sck_write(1);
    delay_us(CS1237_MINI_PLUSE_US);
    sck_write(0);
    delay_us(CS1237_MINI_PLUSE_US);

    drdy_mode_set(CS1237_DOUT_OUTPUT);
    drdy_write(1);	

    return 0;
}

uint8_t CS1237::read_config() {

    uint8_t i;
	uint8_t data = 0;

    _build_data();

    bool success = _wait_drdy_ready(1000); // 等待DRDY准备好，超时时间1000ms (Wait for DRDY to be ready, timeout 1000ms)
    if(!success) {
        SERIAL_ECHOLNPGM("Error: CS1237 DRDY not ready within timeout during config read");
        return -1; // DRDY not ready within timeout
    }

    //29个CLK脉冲 (29 CLK pulses)
    for (int i=0; i<29; i++) {
        sck_write(1);
        delay_us(CS1237_MINI_PLUSE_US);
        sck_write(0);
        delay_us(CS1237_MINI_PLUSE_US);
    }

    //第30~36个脉冲，写配置寄存器 (Pulses 30-36, write configuration register)
    drdy_mode_set(CS1237_DOUT_OUTPUT);
    _write_bit(1);
    _write_bit(0);
    _write_bit(1);
    _write_bit(0);
    _write_bit(1);
    _write_bit(0);
    _write_bit(1);
    _write_bit(0);

    sck_write(1);
    delay_us(CS1237_MINI_PLUSE_US);
    sck_write(0);
    delay_us(CS1237_MINI_PLUSE_US);

    drdy_mode_set(CS1237_DOUT_INTPUT);

    for(i=0; i < 8; i++)
    {
		sck_write(1);
		delay_us(CS1237_MINI_PLUSE_US);
		sck_write(0);
		delay_us(CS1237_MINI_PLUSE_US);	

        data <<= 1;
        if(drdy_read() == 1) {
          data++;
        }	
    }

    sck_write(1);
    delay_us(CS1237_MINI_PLUSE_US);
    sck_write(0);
    delay_us(CS1237_MINI_PLUSE_US);	

    return data;
}

uint32_t CS1237::data_read() {

    uint8_t i=0;
	uint32_t data = 0x0;

    _build_data();

    bool success = _wait_drdy_ready(1000); // 等待DRDY准备好，超时时间1000ms (Wait for DRDY to be ready, timeout 1000ms)
    if(!success) {
        SERIAL_ECHOLNPGM("Error: CS1237 DRDY not ready within timeout during data read");
        return -1; // DRDY not ready within timeout
    }

    // CRITICAL_SECTION_START();
    // 获取24位有效转换 (Read 24-bit valid conversion)
    for (int i=0; i<24; i++) {
        sck_write(1);
        delay_us(CS1237_MINI_PLUSE_US);
        data <<= 1;
        if(drdy_read() == 1){
            data++;
        }
        sck_write(0);
        delay_us(CS1237_MINI_PLUSE_US);
    }

    // 第25~27个脉冲 (Pulses 25-27)
    for(i=0; i<3; i++) {
        sck_write(1);
        delay_us(CS1237_MINI_PLUSE_US);
        sck_write(0);
        delay_us(CS1237_MINI_PLUSE_US);
    }
    // CRITICAL_SECTION_END();

    drdy_mode_set(CS1237_DOUT_INTPUT);

    return data;
}

int CS1237::power_down() {

    sck_write(1);
    delay_us(CS1237_RST_PLUSE_US);
    sck_write(0);
    delay_us(200);

    return 0;
}

CS1237 cs1237; 

#endif // ENV_ALPHA4


