#pragma once

#include <stdint.h>
#include <library/inc/hc32f460_gpio.h>

class CS1237{
public:
    int gpio_init();
    int drdy_mode_set(uint8_t);
    int drdy_write(uint8_t);
    uint8_t drdy_read();
    int sck_write(uint8_t);

    void init();
    int write_config();
    uint8_t read_config();
    uint32_t data_read();
    int power_down();

    void func_init();
    void config_init();
    //cs1237触发状态 (cs1237 trigger state)
    //根据configuration.h设置的Z_MIN_PROBE_ENDSTOP_HIT_STATE (Based on Z_MIN_PROBE_ENDSTOP_HIT_STATE set in configuration.h)
    //如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为HIGH时，返回1为触发，发回0为未触发； (When HIGH: returns 1 = triggered, 0 = not triggered)
    //如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为LOW时，返回0为触发，发回1为未触发； (When LOW: returns 0 = triggered, 1 = not triggered)
    uint8_t trigger();
    //清零，每次回零或调平时，执行一次。 (Zero out; called once each homing or leveling)
    void set_zero();
    void set_threshold(int32_t thr);
    int32_t get_threshold();
    int32_t get_current_value();
    void calc_trigger_state();
    uint32_t get_raw_data();
    float data_deal(int32_t input_data);
    int32_t data_dir(int32_t input_data);
      
    uint8_t homing_flg;
    uint8_t leveling_flg;
    uint8_t endstop_report_flg;
    int32_t cs_zero_val;
private:
    void _write_bit(uint8_t bit);
    void _build_data();
    bool _wait_drdy_ready(uint32_t timeout_ms);

    int32_t cs_data;
    int32_t last_data;
    int32_t cs_throshold;
    int32_t cs_deal_val;
    int32_t cs_current_val;
    int8_t cs_trigger_state;


    stc_port_init_t stcPortInit;
};

extern CS1237 cs1237;



