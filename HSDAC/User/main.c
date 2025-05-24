#include "debug.h"
#include "codec.h"
#include <math.h>
#include "tick.h"
#include "ch32v30x_gpio.h"

typedef struct _QuadOSC {
    float k1;
    float k2;
    float u;
    float v;
} QuadOSC_T;
static void QuadOSC_Init(QuadOSC_T* osc) {
    osc->k1 = 0.0f;
    osc->k2 = 0.0f;
    osc->u =  1.0f;
    osc->v =  0.0f;
}
static void QuadOSC_SetFreq(QuadOSC_T* osc, float omega) {
    osc->k1 = tanf(omega * 0.5f);
    osc->k2 = 2.0f * osc->k1 / (1.0f + osc->k1 * osc->k1);
}
static void QuadOSC_Tick(QuadOSC_T* osc) {
    float w = osc->u - osc->k1 * osc->v;
    osc->v = osc->v + osc->k2 * w;
    osc->u = w - osc->k1 * osc->v;
}

static QuadOSC_T osc_;
static void GenerateSine(StereoSample_T* block, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        QuadOSC_Tick(&osc_);
        float sine = osc_.u;
        float cosine = osc_.v;
        block[i].left = (int16_t)(sine * (INT16_MAX / 2));
        block[i].right = (int16_t)(cosine * (INT16_MAX / 2));
    }
}

static void CheckError(I2CFuture_T* f) {
    if (f->error_code != I2C_EC_NO_ERROR) {
        volatile int i = 1;
        while (1) {
            ++i;
        }
    }
}

static void ES9018K_Init(void) {
    Codec_Init();
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    Delay_Ms(100);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
    Delay_Ms(100);

    Codec_PollWrite(0, 0xf1);
    Codec_PollWrite(1, 0b10000000);
    volatile uint8_t e = Codec_PollRead(64);
    volatile uint32_t i = 0;
    for (;;) {
        ++i;
    }
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    // USART_Printf_Init(115200);

    Tick_Init();

    QuadOSC_Init(&osc_);
    QuadOSC_SetFreq(&osc_, M_PI / 256.0f);

    ES9018K_Init();
    for (;;) {
    }
}
