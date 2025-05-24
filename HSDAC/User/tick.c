#include "tick.h"
#include "ch32v30x_tim.h"
#include "ch32v30x_rcc.h"
// #include "system_ch32v30x.h"

static volatile uint32_t tick_ = 0;

void TIM2_IRQHandler(void) WCH_FAST_INTERRUPT;
void TIM2_IRQHandler(void) {
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    ++tick_;
}

// ========================================
// public
// ========================================
void Tick_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // const uint32_t tim2_freq = SystemCoreClock / 4; // 24MHz
    // prescale * period = 24kHz;
    // period = 1ms

    TIM_TimeBaseInitTypeDef init;
    init.TIM_ClockDivision = TIM_CKD_DIV4;
    init.TIM_CounterMode = TIM_CounterMode_Up;
    init.TIM_RepetitionCounter = 0;
    init.TIM_Period = 1000 - 1;
    init.TIM_Prescaler = 96 - 1;
    TIM_TimeBaseInit(TIM2, &init);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

uint32_t Tick_GetTick(void) {
    return tick_;
}

void Tick_DelayMs(uint32_t ms) {
    uint32_t begin = Tick_GetTick();
    while (tick_ - begin < ms) {}
}
