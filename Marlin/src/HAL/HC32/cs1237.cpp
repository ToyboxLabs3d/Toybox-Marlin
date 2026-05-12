#ifdef ENV_ALPHA4

#include "cs1237.h"

#define CS1237_MINI_PLUSE_US        91//40      
#define CS1237_RST_PLUSE_US         20000//12000    

/*********************************************************************************************************************
 *                                                 CS1237 驱动 (CS1237 driver)
*********************************************************************************************************************/

static void cs1237_write_bit(struct cs1237_dev *cs1237, uint8_t bit) {
    cs1237->cs1237_sck_write(1);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    cs1237->cs1237_drdy_write(bit);
    cs1237->cs1237_sck_write(0);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
}

static void cs1237_build_data(struct cs1237_dev *cs1237) {
    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_OUTPUT);
    cs1237->cs1237_drdy_write(1);
    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_INTPUT);
    cs1237->cs1237_sck_write(0);
}

/************************************************************************
 *  0x5c    //REF输出关闭，输出40hz     PGA=128(有效分辨率为20bit)  通道A  *  (REF off, 40Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x4c    //REF输出关闭，输出10hz     PGA=128(有效分辨率为20bit)  通道A  *  (REF off, 10Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x1c    //REF输出开启，输出40hz     PGA=128(有效分辨率为20bit)  通道A  *  (REF on, 40Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x2c    //REF输出开启，输出640hz    PGA=128(有效分辨率为20bit)  通道A  *  (REF on, 640Hz output, PGA=128 [effective 20-bit], channel A)
 *  0x3c    //REF输出开启，输出1280hz   PGA=128(有效分辨率为20bit)  通道A  *  (REF on, 1280Hz output, PGA=128 [effective 20-bit], channel A)
*************************************************************************/
int cs1237_write_config(struct cs1237_dev *cs1237) {

    uint8_t i;
    uint8_t Data = 0x2c;

    cs1237_build_data(cs1237);

    while(cs1237->cs1237_drdy_read() == 1) {
        // time out..
    }

    //29个CLK脉冲 (29 CLK pulses)
    for (int i=0; i<29; i++) {
        cs1237->cs1237_sck_write(1);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
        cs1237->cs1237_sck_write(0);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    }

    //第30~36个脉冲，写配置寄存器 (Pulses 30-36, write configuration register)
    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_OUTPUT);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 0);
    cs1237_write_bit(cs1237, 0);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 0);
    cs1237_write_bit(cs1237, 1);

    cs1237->cs1237_sck_write(1);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    cs1237->cs1237_sck_write(0);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);

    //第38~45个脉冲，写８位数据 (Pulses 38-45, write 8 bits of data)
	for(i=0; i < 8; i++)
	{
		cs1237->cs1237_sck_write(1);
		cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);

		if(Data & 0x80)
			cs1237->cs1237_drdy_write(1);
		else
			cs1237->cs1237_drdy_write(0);
		
		Data <<= 1;
		
		cs1237->cs1237_sck_write(0);
		cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);	
	}

    //第46个脉冲结束，并释放引脚 (End of pulse 46; release the pin)
    cs1237->cs1237_sck_write(1);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    cs1237->cs1237_sck_write(0);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);

    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_OUTPUT);
    cs1237->cs1237_drdy_write(1);	

    return 0;
}

uint8_t cs1237_read_config(struct cs1237_dev *cs1237) {

    uint8_t i;
	uint8_t Data = 0;

    cs1237_build_data(cs1237);

    while(cs1237->cs1237_drdy_read() == 1) {
        // time out..
    }

    //29个CLK脉冲 (29 CLK pulses)
    for (int i=0; i<29; i++) {
        cs1237->cs1237_sck_write(1);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
        cs1237->cs1237_sck_write(0);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    }

    //第30~36个脉冲，写配置寄存器 (Pulses 30-36, write configuration register)
    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_OUTPUT);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 0);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 0);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 1);
    cs1237_write_bit(cs1237, 0);

    cs1237->cs1237_sck_write(1);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    cs1237->cs1237_sck_write(0);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);

    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_INTPUT);

    for(i=0; i < 8; i++)
    {
		cs1237->cs1237_sck_write(1);
		cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
		cs1237->cs1237_sck_write(0);
		cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);	

        Data <<= 1;
        if(cs1237->cs1237_drdy_read() == 1) {
          Data++;
        }	
    }

    cs1237->cs1237_sck_write(1);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    cs1237->cs1237_sck_write(0);
    cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);	

    return Data;
}

uint32_t cs1237_data_read(struct cs1237_dev *cs1237) {

    uint8_t i=0;
	uint32_t Data = 0x0;

    cs1237_build_data(cs1237);

    while(cs1237->cs1237_drdy_read() == 1) {}

    // 获取24位有效转换 (Read 24-bit valid conversion)
    for (int i=0; i<24; i++) {
        cs1237->cs1237_sck_write(1);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
        Data <<= 1;
        if(cs1237->cs1237_drdy_read() == 1)
            Data++;
        cs1237->cs1237_sck_write(0);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    }

    // 第25~27个脉冲 (Pulses 25-27)
    for(i=0; i<3; i++) {
        cs1237->cs1237_sck_write(1);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
        cs1237->cs1237_sck_write(0);
        cs1237->cs1237_delay_us(CS1237_MINI_PLUSE_US);
    }

    cs1237->cs1237_drdy_mode_set(CS1237_DOUT_INTPUT);

    // if(Data & 0x00800000)
	// {
	// 	Temp = -(((~Data) & 0x007FFFFF)+1);
	// }
	// else
	// {
	// 	Temp = Data & 0x00ffffff;
	// } 
    
    // return Temp;
    return Data;
}

int cs1237_power_down(struct cs1237_dev *cs1237) {

    cs1237->cs1237_sck_write(1);
    cs1237->cs1237_delay_us(CS1237_RST_PLUSE_US);
    cs1237->cs1237_sck_write(0);
    cs1237->cs1237_delay_us(200);

    return 0;
}

#endif // ENV_ALPHA4


