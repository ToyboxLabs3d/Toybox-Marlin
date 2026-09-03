#if defined(ENV_ALPHA4)

#include "dwt.h"
#include <library/inc/hc32f460_pwc.h>

void dwt_init(void)
{
    // 使能DWT功能（通过DEMCR寄存器）(Enable DWT functionality via DEMCR register)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 清零CYCCNT计数器 (Clear CYCCNT counter)
    DWT->CYCCNT = 0;    
    // 使能CYCCNT计数器 (Enable CYCCNT counter)
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 微秒级延时 (Microsecond-level delay)
void dwt_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (F_SYSTEM_CLOCK / 1000000U);
    
    while ((DWT->CYCCNT - start) < cycles);
}

// 毫秒级延时 (Millisecond-level delay)
void dwt_delay_ms(uint32_t ms)
{
    dwt_delay_us(ms * 1000);
}

// 获取纳秒级时间戳（示例，精度取决于系统时钟）(Get nanosecond-level timestamp; example, precision depends on system clock)
uint32_t dwt_ns_tick_get(void)
{
    // 假设系统时钟为200MHz，每个周期约5ns (Assumes system clock is 200MHz, ~5ns per cycle)
    return ((uint32_t)((DWT->CYCCNT) / 200) * 1000);
}

#endif // ENV_ALPHA4