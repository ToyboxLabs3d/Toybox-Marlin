#pragma once

#define CS1237_DOUT_OUTPUT  0x0    
#define CS1237_DOUT_INTPUT  0x1

struct cs1237_dev
{
    int (*cs1237_gpio_init)(void);
    int (*cs1237_drdy_mode_set)(uint8_t);
    int (*cs1237_drdy_write)(uint8_t);
    uint8_t (*cs1237_drdy_read)(void);
    int (*cs1237_sck_write)(uint8_t);
    int (*cs1237_delay_us)(uint32_t);
    int (*cs1237_delay_ns)(uint32_t);   
    
    int32_t cs_zero_val;
    int32_t cs_data;
    int32_t last_data;
    int32_t cs_throshold;
    int32_t cs_deal_val;
    int32_t cs_current_val;
    int8_t cs_trigger_state;

    uint8_t homing_flg;
    uint8_t leveling_flg;
    uint8_t endstop_report_flg;

};



void cs1237_func_init(void);
int cs1237_write_config(struct cs1237_dev *cs1237);
uint8_t cs1237_read_config(struct cs1237_dev *cs1237);
uint32_t cs1237_data_read(struct cs1237_dev *cs1237);
int cs1237_power_down(struct cs1237_dev *cs1237);


