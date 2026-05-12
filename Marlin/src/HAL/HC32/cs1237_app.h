#pragma once

#define CS1237_SCK_PORT     PortA
#define CS1237_SCK_PIN      Pin12
#define CS1237_DRDY_PORT    PortA
#define CS1237_DRDY_PIN     Pin11

extern struct cs1237_dev cs1237; // TODO: import properly

#define CS1237_VREF                     4.89
#define CS1237_GAIN                     128
#define CS1237_ADC_BIT                  8388607
#define CS1237_FIL                      0.9 

#define ZERO_TIMES                      1

void st_timer_init(void);
int cs1237_delay_us(uint32_t us);
int cs1237_gpio_init(void);
int cs1237_drdy_mode_set(uint8_t mode);
int cs1237_drdy_write(uint8_t state);
uint8_t cs1237_drdy_read(void);
int cs1237_sck_write(uint8_t state);
void cs1237_func_init(void);
void cs1237_config_init();
//cs1237触发状态 (cs1237 trigger state)
//根据configuration.h设置的Z_MIN_PROBE_ENDSTOP_HIT_STATE (Based on Z_MIN_PROBE_ENDSTOP_HIT_STATE set in configuration.h)
//如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为HIGH时，返回1为触发，发回0为未触发； (When HIGH: returns 1 = triggered, 0 = not triggered)
//如果Z_MIN_PROBE_ENDSTOP_HIT_STATE为LOW时，返回0为触发，发回1为未触发； (When LOW: returns 0 = triggered, 1 = not triggered)
uint8_t cs1237_trigger();
//清零，每次回零或调平时，执行一次。 (Zero out; called once each homing or leveling)
void cs1237_set_zero(struct cs1237_dev *cs1237);
void cs1237_set_threshold(int32_t thr);
int32_t cs1237_get_threshold();
int32_t cs1237_get_current_value();
void calc_cs1237_trigger_state();
