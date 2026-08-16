#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include "zf_driver_exti.h"
#include "Encoder.h"

#define PULSE_PER_CIRCLE   (500 * 20)    // 500线 * 20减速比 = 输出轴一圈的脉冲数

//==========================================
// 编码器右 — TIMG8 硬件 QEI (B21=A相, B22=B相)
//==========================================
static int32_t g_encoderR_count;

void EncoderR_Init(void)
{
    g_encoderR_count = 0;

    afio_init(B21, GPI, GPIO_AF3, GPI_PULL_UP);
    afio_init(B22, GPI, GPIO_AF3, GPI_PULL_UP);

    DL_TimerG_reset(TIMG8);
    DL_TimerG_enablePower(TIMG8);
    DL_TimerG_enableClock(TIMG8);

    TIMG8->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    TIMG8->CLKDIV = 0;

    DL_TimerG_configQEI(TIMG8,
        DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT,
        DL_TIMER_CC_0_INDEX);

    DL_TimerG_setLoadValue(TIMG8, 0xFFFF);
    TIMG8->COUNTERREGS.CTR = 0;
    DL_TimerG_startCounter(TIMG8);
}

int32_t EncoderR_GetCount(void)
{
    static int16_t last_ctr = 0;
    static uint8_t first = 1;
    int16_t now = (int16_t)TIMG8->COUNTERREGS.CTR;

    if(first)
    {
        first = 0;
        last_ctr = now;
        return 0;
    }

    int16_t delta = now - last_ctr;   // 2's complement auto-handles overflow
    last_ctr = now;
    g_encoderR_count += delta;
    return g_encoderR_count;
}

void EncoderR_Clear(void)
{
    TIMG8->COUNTERREGS.CTR = 0;
}

//==========================================
// 编码器左 — exti 软件 2 倍频 (B26=A相, B27=B相)
//==========================================
static int32_t g_encoderL_count;

static void EncoderL_exti_handler(uint32_t event, void *ptr)
{
    static uint8_t last_a = 1, last_b = 1;
    uint8_t a = gpio_get_level(B26);
    uint8_t b = gpio_get_level(B27);

    if(a != last_a)
    {
        if(a == b) g_encoderL_count++;
        else       g_encoderL_count--;
    }
    if(b != last_b)
    {
        if(a != b) g_encoderL_count++;
        else       g_encoderL_count--;
    }

    last_a = a;
    last_b = b;
}

void EncoderL_Init(void)
{
    g_encoderL_count = 0;

    gpio_init(B26, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(B27, GPI, GPIO_HIGH, GPI_PULL_UP);

    exti_init(B26, EXTI_TRIGGER_BOTH, EncoderL_exti_handler, NULL);
    exti_init(B27, EXTI_TRIGGER_BOTH, EncoderL_exti_handler, NULL);
}

int32_t EncoderL_GetCount(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t ret = g_encoderL_count;
    __set_PRIMASK(primask);
    return ret;
}

void EncoderL_Clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_encoderL_count = 0;
    __set_PRIMASK(primask);
}

//==========================================
// 计算 RPM
//==========================================
float CalcRPM(int32_t pulse_diff, uint32_t time_ms)
{
    const float PULSE_PER_REV = (float)PULSE_PER_CIRCLE * 4.0f;
    float rev = (float)pulse_diff / PULSE_PER_REV;
    float rpm = rev / ((float)time_ms / 60000.0f);
    return rpm;
}
