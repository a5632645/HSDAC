#include "codec.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_spi.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_i2c.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "usb/ch32v30x_usbhs_device.h"

#define I2S_DMA_BLOCK_SIZE 256
#define I2S_DMA_BUFFER_SIZE (I2S_DMA_BLOCK_SIZE * 2)
#define I2S_DMA_BUFFER_SIZE_MASK (I2S_DMA_BLOCK_SIZE * 2 - 1)
static StereoSample_T i2s_dma_buffer_[I2S_DMA_BUFFER_SIZE] = {0};
static volatile int32_t last_i2s_dma_wpos_ = 0;
static volatile bool transfer_error_ = false;
#define UAC_BUFFER_LEN 2048
#define UAC_BUFFER_LEN_MASK 2047
#define UAC_WPOS_INIT                 (UAC_BUFFER_LEN / 2)
#define UAC_BUFFER_LEN_UP_THRESHOLD   (UAC_BUFFER_LEN * 60000 / 100000)
#define UAC_BUFFER_LEN_DOWN_THRESHOLD (UAC_BUFFER_LEN * 40000 / 100000)
static StereoSample_T uac_buffer_[UAC_BUFFER_LEN] = {0};
static volatile uint32_t uac_buf_wpos_ = UAC_WPOS_INIT;
static volatile uint32_t uac_buf_rpos = 0;

#define I2S_PRESCALE_ODD_MASK 0x100
#define I2S_PRESCALE_DIV_MASK 0xff
#define I2S_PRESCALE_MASK     0x1ff

static volatile uint32_t num_usb = 0;
static volatile uint32_t num_dma = 0;
static volatile uint32_t num_dma_cplt_tx = 0;
static volatile uint32_t sample_rate_ = 48000;
#define FEEDBACK_REPORT_PERIOD 64
static volatile uint32_t feedback_report_counter_ = 0;
#define DMA_FREQUENCY_MEAURE_PERIOD 80
static volatile uint32_t dma_frequency_meausure_counter_ = 0;
static volatile float raw_mesured_dma_sample_rate_ = 0;
uint32_t mesured_dma_sample_rate_ = 0;
uint32_t mesured_usb_sample_rate_ = 0;

volatile uint32_t max_uac_len_ever = 0;
volatile uint32_t min_uac_len_ever = 0xffffffff;
#define MIN(a, b) ((a) > (b) ? (b) : (a))

static void DMA_Tx_Init(DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA_CHx);

    DMA_InitTypeDef DMA_InitStructure={0};
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    // dma interrupt
    NVIC_InitTypeDef dma_nvic;
    dma_nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    dma_nvic.NVIC_IRQChannelCmd = ENABLE;
    dma_nvic.NVIC_IRQChannelPreemptionPriority = 2;
    dma_nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&dma_nvic);

    DMA_ClearITPendingBit(DMA1_IT_TC5);
    // DMA_ClearITPendingBit(DMA1_IT_HT5);
    DMA_ClearITPendingBit(DMA1_IT_TE5);

    // DMA_ITConfig(DMA1_Channel5, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, ENABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC | DMA_IT_TE, ENABLE);
}

static void I2S2_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    SPI_I2S_DeInit(SPI2);

    GPIO_InitTypeDef GPIO_InitStructure={0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2S_InitTypeDef I2S_InitStructure={0};
    I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
    I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
    I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_32b;
    I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
    I2S_InitStructure.I2S_AudioFreq = I2S_AudioFreq_48k;
    I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
    I2S_Init(SPI2, &I2S_InitStructure);

    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
    I2S_Cmd(SPI2, ENABLE);
}

static void I2C2_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    I2C_DeInit(I2C2);

    GPIO_InitTypeDef init;
    init.GPIO_Mode = GPIO_Mode_AF_OD;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    init.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &init);

    I2C_InitTypeDef i2c_init = {
        .I2C_ClockSpeed = 80000,
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_DutyCycle = I2C_DutyCycle_16_9,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
    };
    I2C_Init(I2C2, &i2c_init);
    I2C_Cmd(I2C2, ENABLE);
}

void DMA1_Channel5_IRQHandler(void) WCH_FAST_INTERRUPT;
void DMA1_Channel5_IRQHandler(void) {
    if (DMA_GetFlagStatus(DMA1_FLAG_TE5) == SET) {
        DMA_ClearFlag(DMA1_FLAG_TE5);
        DMA_Cmd(DMA1_Channel5, DISABLE);
        I2S_Cmd(SPI2, DISABLE);
        transfer_error_ = true;
    }
    if (DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET) {
        DMA_ClearFlag(DMA1_FLAG_TC5);
        NVIC_DisableIRQ(USBHS_IRQn);
        ++num_dma_cplt_tx;
        NVIC_EnableIRQ(USBHS_IRQn);
    }
}

// ========================================
// public
// ========================================
static int32_t Swap16(int32_t x) {
    uint32_t r = x;
    uint32_t up = r & 0xffff;
    uint32_t down = r >> 16;
    return down | (up << 16);
}

void Codec_Init(void) {
    I2S2_Init();
    DMA_Tx_Init(DMA1_Channel5, (u32)&SPI2->DATAR, (u32)&i2s_dma_buffer_, I2S_DMA_BUFFER_SIZE * sizeof(StereoSample_T) / sizeof(uint16_t));
    I2C2_Init();

    // PA1/PA5 -> RESETB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef init;
    init.GPIO_Mode = GPIO_Mode_AIN;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    init.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &init);
    init.GPIO_Mode = GPIO_Mode_Out_PP;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    init.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &init);

    for (uint32_t i = 0; i < I2S_DMA_BUFFER_SIZE; ++i) {
        float p = (float)i * M_PI * 2 / (float)(I2S_DMA_BUFFER_SIZE);
        float s = sinf(p);
        float cs = cosf(p);
        i2s_dma_buffer_[i].left = Swap16(s * (1 << 24));
        i2s_dma_buffer_[i].right = Swap16(cs * (1 << 24));
    }

    DMA_Cmd(DMA1_Channel5, ENABLE);
}

void Codec_DeInit(void) {
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_DeInit(DMA1_Channel5);
    I2S_Cmd(SPI2, DISABLE);
    SPI_I2S_DeInit(SPI2);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, DISABLE);
}

bool Codec_IsTransferError(void) {
    return transfer_error_;
}

uint32_t Codec_GetBlockSize(void) {
    return I2S_DMA_BLOCK_SIZE;
}

void Codec_PollWrite(uint8_t reg, uint8_t val) {
    while (I2C_GetFlagStatus (I2C2, I2C_FLAG_BUSY) != RESET) {}
    I2C_GenerateSTART (I2C2, ENABLE);
    while (!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {}
    I2C_Send7bitAddress (I2C2, 0x90, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {}
    while (I2C_GetFlagStatus (I2C2, I2C_FLAG_TXE) == RESET) {}
    I2C_SendData (I2C2, reg);
    while (I2C_GetFlagStatus (I2C2, I2C_FLAG_TXE) == RESET) {}
    I2C_SendData (I2C2, val);
    while (I2C_GetFlagStatus (I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET) {}
    I2C_GenerateSTOP (I2C2, ENABLE);
    Delay_Ms(20);
}

uint8_t Codec_PollRead(uint8_t reg) {
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET) {}
    I2C_GenerateSTART(I2C2, ENABLE);
    while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) == RESET) {}
    I2C_Send7bitAddress(I2C2, 0x90, I2C_Direction_Transmitter);
    while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET) {}
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET) {}
    I2C_SendData(I2C2, reg);
    while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET) {}
    I2C_GenerateSTART(I2C2, ENABLE);
    while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) == RESET) {}
    I2C_Send7bitAddress(I2C2, 0x90, I2C_Direction_Receiver);
    while (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == RESET) {}
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET) {}
    uint8_t val = I2C_ReceiveData(I2C2);
    I2C_GenerateSTOP(I2C2, ENABLE);
    return val;
}

// ---------- clock sync ----------
void Codec_Start (void) {
    uac_buf_wpos_ = UAC_WPOS_INIT;
    uac_buf_rpos = 0;
}

void Codec_Stop(void) {
    uac_buf_rpos = uac_buf_wpos_;
    memset(i2s_dma_buffer_, 0, sizeof(i2s_dma_buffer_));
}

uint32_t Codec_GetDMASize (void) {
    return I2S_DMA_BUFFER_SIZE;
}

uint32_t Codec_GetDMAHalfSize (void) {
    return I2S_DMA_BLOCK_SIZE;
}

bool Codec_IsDMAStart (void) {
    return true;
}

void Codec_WriteUACBuffer (const uint8_t* ptr, uint32_t len) {
    int32_t curr_dma_pos = (I2S_DMA_BUFFER_SIZE * 4 - DMA_GetCurrDataCounter(DMA1_Channel5)) / 4;
    uint32_t dma_can_write = (curr_dma_pos - last_i2s_dma_wpos_) & I2S_DMA_BUFFER_SIZE_MASK;

    uint32_t uac_len = (uac_buf_wpos_ - uac_buf_rpos + UAC_BUFFER_LEN) & UAC_BUFFER_LEN_MASK;
    uint32_t uac_to_dma = uac_len > dma_can_write ? dma_can_write : uac_len;
    uac_len -= uac_to_dma;
    dma_can_write -= uac_to_dma;
    while (uac_to_dma--) {
        StereoSample_T sample = uac_buffer_[uac_buf_rpos];
        uac_buf_rpos = (uac_buf_rpos + 1) & UAC_BUFFER_LEN_MASK;
        i2s_dma_buffer_[last_i2s_dma_wpos_] = sample;
        last_i2s_dma_wpos_ = (last_i2s_dma_wpos_ + 1) & I2S_DMA_BUFFER_SIZE_MASK;
    }

    uint32_t num_input_stereo_samples = len / sizeof(StereoSample_T);
    num_usb += num_input_stereo_samples;
    uint32_t usb_to_dma = num_input_stereo_samples > dma_can_write ? dma_can_write : num_input_stereo_samples;
    dma_can_write -= usb_to_dma;
    num_input_stereo_samples -= usb_to_dma;
    const uint32_t* src_ptr = (const uint32_t*)ptr;
    while (usb_to_dma--) {
        uint32_t left = Swap16(*src_ptr);
        ++src_ptr;
        uint32_t right = Swap16(*src_ptr);
        ++src_ptr;
        i2s_dma_buffer_[last_i2s_dma_wpos_].left = left;
        i2s_dma_buffer_[last_i2s_dma_wpos_].right = right;
        last_i2s_dma_wpos_ = (last_i2s_dma_wpos_ + 1) & I2S_DMA_BUFFER_SIZE_MASK;
    }

    uint32_t can_write = UAC_BUFFER_LEN_MASK - uac_len;
    if (num_input_stereo_samples > can_write) num_input_stereo_samples = can_write;
    while (num_input_stereo_samples--) {
        uac_buffer_[uac_buf_wpos_].left = Swap16(*src_ptr);
        ++src_ptr;
        uac_buffer_[uac_buf_wpos_].right = Swap16(*src_ptr);
        ++src_ptr;
        ++uac_buf_wpos_;
        uac_buf_wpos_ &= UAC_BUFFER_LEN_MASK;
    }
}

uint32_t Codec_GetUACBufferLen (void) {
    return (uac_buf_wpos_ - uac_buf_rpos + UAC_BUFFER_LEN) & UAC_BUFFER_LEN_MASK;
}

uint32_t Codec_GetDMALen(void) {
    int32_t curr_dma_counter = DMA_GetCurrDataCounter(DMA1_Channel5);
    int32_t count = num_dma - curr_dma_counter + I2S_DMA_BUFFER_SIZE * 4 * num_dma_cplt_tx;
    num_dma = curr_dma_counter;
    num_dma_cplt_tx = 0;
    return count;
}

static void I2S2_PrescaleConfig(uint32_t v) {
    uint16_t odd = (v & 1) << 8;
    uint16_t div = v >> 1;
    uint16_t mlck = 1 << 9;
    SPI2->I2SPR = odd | div | mlck;
}

float report_fs_ = 48000.0f;
static float timeing = 1.0f;
void Codec_SetSampleRate(uint32_t sample_rate) {
    sample_rate_ = sample_rate;
    mesured_dma_sample_rate_ = sample_rate;
    min_uac_len_ever = 2048 / 2;
    max_uac_len_ever = 2048 / 2;
    report_fs_ = sample_rate;
    raw_mesured_dma_sample_rate_ = sample_rate;

    switch (sample_rate) {
    case 96000:
        I2S2_PrescaleConfig(8);
        timeing = 1.0f; // 1sec
        break;
    case 192000:
        I2S2_PrescaleConfig(4);
        timeing = 2.0f; // 0.25sec
        break;
    case 48000:
    default:
        I2S2_PrescaleConfig(16);
        timeing = 0.5f; // 2sec
        break;
    }
}

void Codec_MeasureSampleRateAndReportFeedback(void) {
    uint32_t uac_len = Codec_GetUACBufferLen();
    if (uac_len < min_uac_len_ever) min_uac_len_ever = uac_len;
    if (uac_len > max_uac_len_ever) max_uac_len_ever = uac_len;

    ++dma_frequency_meausure_counter_;
    if (dma_frequency_meausure_counter_ >= DMA_FREQUENCY_MEAURE_PERIOD) {
        // 10ms
        dma_frequency_meausure_counter_ = 0;
        uint32_t dma_bck = Codec_GetDMALen();
        float fs = dma_bck * (1000 / (DMA_FREQUENCY_MEAURE_PERIOD / 8)) / 4;
        if (fs > (sample_rate_ + 2000)) fs = sample_rate_ + 2000;
        else if (fs < (sample_rate_ - 2000)) fs = sample_rate_ - 2000;
        raw_mesured_dma_sample_rate_ = raw_mesured_dma_sample_rate_ * 0.95f + fs * 0.05f;
        mesured_dma_sample_rate_ = raw_mesured_dma_sample_rate_;
        mesured_usb_sample_rate_ = num_usb * (1000 / (DMA_FREQUENCY_MEAURE_PERIOD / 8));
        num_usb = 0;
    }

    ++feedback_report_counter_;
    if (feedback_report_counter_ >= FEEDBACK_REPORT_PERIOD) {
        // 8ms
        feedback_report_counter_ = 0;
        int32_t uac_len = Codec_GetUACBufferLen();
        int32_t diff = (uac_len - UAC_BUFFER_LEN / 2);
        if (diff > -10 && diff < 0) diff = -10;
        if (diff > 0 && diff < 10) diff = 10;
        float fb = raw_mesured_dma_sample_rate_ - diff * timeing;
        report_fs_ = report_fs_ * 0.99f + fb * 0.01f;
        USBUAC_WriteFeedback(report_fs_);
    }
}
