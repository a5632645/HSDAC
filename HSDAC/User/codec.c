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
#include "tick.h"
#include "usb/ch32v30x_usbhs_device.h"

#define I2S_DMA_BLOCK_SIZE 192
#define I2S_DMA_BUFFER_SIZE (I2S_DMA_BLOCK_SIZE * 2)
static StereoSample_T i2s_dma_buffer_[I2S_DMA_BUFFER_SIZE] = {0};
static volatile bool transfer_error_ = false;

// clock sync
#define UAC_BUFFER_LEN 2048
#define UAC_BUFFER_LEN_MASK 2047
#define UAC_WPOS_INIT                 (UAC_BUFFER_LEN / 2)
#define UAC_BUFFER_LEN_UP_THRESHOLD   (UAC_BUFFER_LEN * 60000 / 100000)
#define UAC_BUFFER_LEN_DOWN_THRESHOLD (UAC_BUFFER_LEN * 40000 / 100000)
static volatile int32_t dma_counter_ = 0;
static StereoSample_T uac_buffer_[UAC_BUFFER_LEN] = {0};
static uint32_t uac_buf_wpos_ = UAC_WPOS_INIT;
static uint32_t uac_buf_rpos = 0;

#define I2S_PRESCALE_ODD_MASK 0x100
#define I2S_PRESCALE_DIV_MASK 0xff
#define I2S_PRESCALE_MASK     0x1ff

uint32_t num_usb = 0;
uint32_t num_dma = 0;
uint32_t num_dma_cplt_tx = 0;
static uint32_t sample_rate_ = 0;
// float resample_ratio_ = 1.0f;
// float resample_ratio_inc_ = 0.0f;
// uint32_t resample_freq_inc_ = 5;
// static float resample_phase_ = 0.0f;

// #define BUFFER_STATE_OVERLOAD 1
// #define BUFFER_STATE_UNDERLOAD 2
// static uint8_t last_buffer_state_ = 0;
// static uint8_t ccc = 0;
#define FEEDBACK_REPORT_PERIOD 8
static uint8_t feedback_report_counter_ = 0;
#define DMA_FREQUENCY_MEAURE_PERIOD 80
static uint8_t dma_frequency_meausure_counter_ = 0;
uint32_t mesured_dma_sample_rate_ = 0;

volatile uint32_t max_uac_len_ever = 0;
volatile uint32_t min_uac_len_ever = 0xffffffff;
#define MIN(a, b) ((a) > (b) ? (b) : (a))

// static void IncPrescale(uint16_t* s) {
//     uint16_t tmp = *s;
//     uint16_t odd = 0;
//     uint16_t div = tmp & I2S_PRESCALE_DIV_MASK;
//     if (tmp & I2S_PRESCALE_ODD_MASK) {
//         odd = 0;
//         ++div;
//     }
//     else {
//         odd = I2S_PRESCALE_ODD_MASK;
//     }
//     tmp = (tmp & ~I2S_PRESCALE_MASK) | odd | div;
//     *s = tmp;
// }

// static void DecPrescale(uint16_t* s) {
//     uint16_t tmp = *s;
//     uint16_t odd = 0;
//     uint16_t div = tmp & I2S_PRESCALE_DIV_MASK;
//     if (tmp & I2S_PRESCALE_ODD_MASK) {
//         odd = 0;
//     }
//     else {
//         odd = I2S_PRESCALE_ODD_MASK;
//         --div;
//     }
//     tmp = (tmp & ~I2S_PRESCALE_MASK) | odd | div;
//     *s = tmp;
// }

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
    DMA_ClearITPendingBit(DMA1_IT_HT5);
    DMA_ClearITPendingBit(DMA1_IT_TE5);

    DMA_ITConfig(DMA1_Channel5, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, ENABLE);
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

    // NVIC_InitTypeDef nvic;
    // nvic.NVIC_IRQChannel = I2C2_ER_IRQn;
    // nvic.NVIC_IRQChannelCmd = ENABLE;
    // nvic.NVIC_IRQChannelPreemptionPriority = 2;
    // nvic.NVIC_IRQChannelSubPriority = 0;
    // NVIC_Init(&nvic);
    // nvic.NVIC_IRQChannel = I2C2_EV_IRQn;
    // NVIC_Init(&nvic);
    
    // I2C_ITConfig(I2C2, I2C_IT_EVT, ENABLE);
    // I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
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

        StereoSample_T* block_to_fill_ = &i2s_dma_buffer_[I2S_DMA_BLOCK_SIZE];
        uint32_t uac_len = Codec_GetUACBufferLen();
        uint32_t read_len = uac_len > I2S_DMA_BLOCK_SIZE ? I2S_DMA_BLOCK_SIZE : uac_len;
        while (read_len--) {
            *block_to_fill_ = uac_buffer_[uac_buf_rpos];
            ++block_to_fill_;
            ++uac_buf_rpos;
            uac_buf_rpos &= UAC_BUFFER_LEN_MASK;
        }

        // uint32_t bck_rpos = uac_buf_rpos;
        // uint32_t resampled_len = 0;

        // // resample buffer
        // for (uint32_t i = 0; i < I2S_DMA_BLOCK_SIZE; ++i) {
        //     block_to_fill_->left = uac_buffer_[bck_rpos].left;
        //     block_to_fill_->right = uac_buffer_[bck_rpos].right;
        //     ++block_to_fill_;
        //     resample_phase_ += resample_ratio_;
        //     if (resample_phase_ >= 1.0f) {
        //         uint32_t iinc = (uint32_t)resample_phase_;
        //         bck_rpos = (bck_rpos + iinc) & UAC_BUFFER_LEN_MASK;
        //         resample_phase_ -= iinc;
        //         resampled_len += iinc;
        //     }
        // }

        // // process len
        // uint32_t len = Codec_GetUACBufferLen();
        // if (resampled_len > len) {
        //     resampled_len = len;
        // }
        // uac_buf_rpos = (uac_buf_rpos + resampled_len) & UAC_BUFFER_LEN_MASK;
    }
    if (DMA_GetFlagStatus(DMA1_FLAG_HT5) == SET) {
        DMA_ClearFlag(DMA1_FLAG_HT5);
        NVIC_DisableIRQ(USBHS_IRQn);

        StereoSample_T* block_to_fill_ = &i2s_dma_buffer_[0];
        uint32_t uac_len = Codec_GetUACBufferLen();
        uint32_t read_len = uac_len > I2S_DMA_BLOCK_SIZE ? I2S_DMA_BLOCK_SIZE : uac_len;
        while (read_len--) {
            *block_to_fill_ = uac_buffer_[uac_buf_rpos];
            ++block_to_fill_;
            ++uac_buf_rpos;
            uac_buf_rpos &= UAC_BUFFER_LEN_MASK;
        }

        // uint32_t bck_rpos = uac_buf_rpos;
        // uint32_t resampled_len = 0;

        // // resample buffer
        // for (uint32_t i = 0; i < I2S_DMA_BLOCK_SIZE; ++i) {
        //     block_to_fill_->left = uac_buffer_[bck_rpos].left;
        //     block_to_fill_->right = uac_buffer_[bck_rpos].right;
        //     ++block_to_fill_;
        //     resample_phase_ += resample_ratio_;
        //     if (resample_phase_ >= 1.0f) {
        //         uint32_t iinc = (uint32_t)resample_phase_;
        //         bck_rpos = (bck_rpos + iinc) & UAC_BUFFER_LEN_MASK;
        //         resample_phase_ -= iinc;
        //         resampled_len += iinc;
        //     }
        // }

        // // process len
        // uint32_t len = Codec_GetUACBufferLen();
        // if (resampled_len > len) {
        //     resampled_len = len;
        // }
        // uac_buf_rpos = (uac_buf_rpos + resampled_len) & UAC_BUFFER_LEN_MASK;
    }

    // process underload/overload
    // if (Tick_GetTick() - ccc > 20) {
    //     ccc = Tick_GetTick();

    //     uint32_t uac_len = Codec_GetUACBufferLen();
    //     if (uac_len > UAC_BUFFER_LEN_UP_THRESHOLD) {
    //         // if (last_buffer_state_ == BUFFER_STATE_OVERLOAD) {
    //         //     // 连续过载，直接增加重采样系数
    //         //     resample_ratio_ += resample_ratio_inc_;
    //         // }
    //         // else if (last_buffer_state_ == BUFFER_STATE_UNDERLOAD) {
    //         //     // 欠载之后又过载，减小系数增加，增加系数
    //         //     resample_freq_inc_ /= 2;
    //         //     resample_ratio_inc_ = (float)resample_freq_inc_ / (float)sample_rate_;
    //         //     resample_ratio_ += resample_ratio_inc_;
    //         //     last_buffer_state_ = BUFFER_STATE_OVERLOAD;
    //         // }
    //         // else {
    //         //     // 我不知道
    //         //     resample_ratio_ += resample_ratio_inc_;
    //         // }
    //         // last_buffer_state_ = BUFFER_STATE_OVERLOAD;
    //         resample_ratio_ += resample_ratio_inc_;
    //     }
    //     else if (uac_len < UAC_BUFFER_LEN_DOWN_THRESHOLD) {
    //         // if (last_buffer_state_ == BUFFER_STATE_OVERLOAD) {
    //         //     // 过载之后又欠载，减小系数增加，减小系数
    //         //     resample_freq_inc_ /= 2;
    //         //     resample_ratio_inc_ = (float)resample_freq_inc_ / (float)sample_rate_;
    //         //     resample_ratio_ -= resample_ratio_inc_;
    //         // }
    //         // else if (last_buffer_state_ == BUFFER_STATE_UNDERLOAD) {
    //         //     // 连续欠载，直接减小重采样系数
    //         //     resample_ratio_ -= resample_ratio_inc_;
    //         // }
    //         // else {
    //         //     // 我不知道
    //         //     resample_ratio_ -= resample_ratio_inc_;
    //         // }
    //         // last_buffer_state_ = BUFFER_STATE_UNDERLOAD;
    //         resample_ratio_ -= resample_ratio_inc_;
    //     }
    // }

    // resample_ratio_ = resample_ratio_ * 0.999f + 1.0f * 0.001f;

    NVIC_EnableIRQ(USBHS_IRQn);
}

static I2CFuture_T* i2c_future_ = NULL;
void I2C2_EV_IRQHandler(void) WCH_FAST_INTERRUPT;
void I2C2_EV_IRQHandler(void) {
    if (I2C_GetITStatus(I2C2, I2C_IT_SB) == SET) {
        switch (i2c_future_->state) {
        case I2C_FUTURE_INIT_TX:
            I2C_Send7bitAddress(I2C2, 0x90, I2C_Direction_Transmitter);
            i2c_future_->state = I2C_FUTURE_ADDRESS_SENDED_TX;
            break;
        case I2C_FUTURE_INIT_RX:
            I2C_Send7bitAddress(I2C2, 0x90, I2C_Direction_Transmitter);
            i2c_future_->state = I2C_FUTURE_ADDRESS_SENDED_RX;
            break;
        case I2C_FUTURE_INIT_RX_RX:
            I2C_Send7bitAddress(I2C2, 0x90, I2C_Direction_Receiver);
            i2c_future_->state = I2C_FUTURE_ADDRESS_SENDED_RX_RX;
            break;
        }
    }
    else if (I2C_GetITStatus(I2C2, I2C_IT_ADDR) == SET) {
        volatile uint16_t no_optimise = I2C_ReadRegister(I2C2, I2C_Register_STAR1);
        no_optimise = I2C_ReadRegister(I2C2, I2C_Register_STAR2);
        (void)no_optimise;
    }
    else if (I2C_GetITStatus(I2C2, I2C_IT_TXE) == SET) {
        switch (i2c_future_->state) {
        case I2C_FUTURE_ADDRESS_SENDED_TX:
            I2C_SendData(I2C2, i2c_future_->reg);
            i2c_future_->state = I2C_FUTURE_REG_SENDED_TX;
            break;
        case I2C_FUTURE_ADDRESS_SENDED_RX:
            I2C_SendData(I2C2, i2c_future_->reg);
            i2c_future_->state = I2C_FUTURE_REG_SENDED_RX;
            break;
        case I2C_FUTURE_REG_SENDED_RX:
            I2C_GenerateSTART(I2C2, ENABLE);
            I2C_SendData(I2C2, 0xff);
            i2c_future_->state = I2C_FUTURE_INIT_RX_RX;
            break;
        case I2C_FUTURE_REG_SENDED_TX:
            I2C_SendData(I2C2, i2c_future_->val);
            i2c_future_->state = I2C_FUTURE_VAL_SENDED_TX;
            break;
        case I2C_FUTURE_VAL_SENDED_TX:
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_SendData(I2C2, 0xff);
            i2c_future_->state = I2C_FUTURE_COMPLETED;
            i2c_future_->error_code = I2C_EC_NO_ERROR;
            break;
        default:
            break;
        }
    }
    else if (I2C_GetITStatus(I2C2, I2C_IT_RXNE) == SET) {
        if (i2c_future_->state == I2C_FUTURE_ADDRESS_SENDED_RX_RX) {
            i2c_future_->val = I2C_ReceiveData(I2C2);
            i2c_future_->state = I2C_FUTURE_COMPLETED;
            i2c_future_->error_code = I2C_EC_NO_ERROR;
            I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Next);
            I2C_GenerateSTOP(I2C2, ENABLE);
        }
    }
}

void I2C2_ER_IRQHandler(void) WCH_FAST_INTERRUPT;
void I2C2_ER_IRQHandler(void) {
    if (I2C_GetITStatus(I2C2, I2C_IT_AF) == SET) {
        i2c_future_->error_code = I2C_EC_NACK;
        I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
    }
    else {
        i2c_future_->error_code = I2C_EC_UNKOWN;
    }
    I2C_GenerateSTOP(I2C2, ENABLE);
    i2c_future_->state = I2C_FUTURE_COMPLETED;
}

// ========================================
// public
// ========================================
static int32_t Swap16(int32_t x) {
    volatile int32_t ret = 0;
    volatile int16_t* a = &x;
    volatile int16_t* b = &ret;
    b[0] = a[1];
    b[1] = a[0];
    return ret;
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
        i2s_dma_buffer_[i].left = Swap16(s * (1 << 31));
        i2s_dma_buffer_[i].right = Swap16(cs * (1 << 31));
    }
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

void Codec_Write(I2CFuture_T* pstate) {
    i2c_future_ = pstate;
    i2c_future_->state = I2C_FUTURE_INIT_TX;
    I2C_GenerateSTART(I2C2, ENABLE);
}

void Codec_Read(I2CFuture_T* pstate) {
    i2c_future_ = pstate;
    i2c_future_->state = I2C_FUTURE_INIT_RX;
    I2C_GenerateSTART(I2C2, ENABLE);
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
void Codec_ClockSync_Reset (void) {
    dma_counter_ = I2S_DMA_BUFFER_SIZE * sizeof(StereoSample_T) / sizeof(uint16_t);
    num_dma = dma_counter_;
}

uint32_t Codec_ClockSync_GetNumRead (void) {
    int32_t curr_dma_counter = DMA_GetCurrDataCounter(DMA1_Channel5);
    int32_t count = (dma_counter_ - curr_dma_counter) / 4;
    if (count < 0) {
        count += I2S_DMA_BUFFER_SIZE;
    }
    dma_counter_ = curr_dma_counter;
    return count;
}

void Codec_SetResampleRatio (float ratio) {
}

void Codec_Start (void) {
    Codec_ClockSync_Reset();
    DMA_Cmd(DMA1_Channel5, ENABLE);
}

void Codec_Stop(void) {
    DMA_Cmd(DMA1_Channel5, DISABLE);
    uac_buf_wpos_ = UAC_WPOS_INIT;
    uac_buf_rpos = 0;
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
    uint32_t num_input_stereo_samples = len / sizeof(StereoSample_T);
    num_usb += num_input_stereo_samples;

    uint32_t uac_len = (uac_buf_wpos_ - uac_buf_rpos + UAC_BUFFER_LEN) & UAC_BUFFER_LEN_MASK;
    if (uac_len < min_uac_len_ever) min_uac_len_ever = uac_len;

    uint32_t can_write = UAC_BUFFER_LEN_MASK - uac_len;
    if (num_input_stereo_samples > can_write) num_input_stereo_samples = can_write;
    const uint32_t* src_ptr = (const uint32_t*)ptr;
    while (num_input_stereo_samples--) {
        uac_buffer_[uac_buf_wpos_].left = Swap16(*src_ptr);
        ++src_ptr;
        uac_buffer_[uac_buf_wpos_].right = Swap16(*src_ptr);
        ++src_ptr;
        ++uac_buf_wpos_;
        uac_buf_wpos_ &= UAC_BUFFER_LEN_MASK;
    }

    uac_len = (uac_buf_wpos_ - uac_buf_rpos + UAC_BUFFER_LEN) & UAC_BUFFER_LEN_MASK;
    if (uac_len > max_uac_len_ever) max_uac_len_ever = uac_len;
}

uint32_t Codec_GetUACBufferLen (void) {
    return (uac_buf_wpos_ - uac_buf_rpos + UAC_BUFFER_LEN) & UAC_BUFFER_LEN_MASK;
}

void Codec_CheckBuffer(void) {
}

uint32_t Codec_GetDMALen(void) {
    int32_t curr_dma_counter = DMA_GetCurrDataCounter(DMA1_Channel5);
    int32_t count = num_dma - curr_dma_counter + I2S_DMA_BUFFER_SIZE * 4 * num_dma_cplt_tx;
    num_dma = curr_dma_counter;
    num_dma_cplt_tx = 0;
    return count;
}

static void I2S2_PrescaleConfig(uint32_t v) {
    I2S_Cmd(SPI2, DISABLE);
    uint16_t odd = (v & 1) << 8;
    uint16_t div = v >> 1;
    uint16_t mlck = 1 << 9;
    SPI2->I2SPR = odd | div | mlck;
    I2S_Cmd(SPI2, ENABLE);
}

// pll3的倍频系数*2的值，正确的值需要/2
static const uint32_t kPLL3MulTable[] = {
    5, 25, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 40
};
#define MIN_I2S_PRESCALE 2
#define MAX_I2S_PRESCALE 511
#define NUM_PLL3_MUL 14
void Codec_SetSampleRate(uint32_t sample_rate) {
    sample_rate_ = sample_rate;
    mesured_dma_sample_rate_ = sample_rate;
    // resample_ratio_inc_ = (float)resample_freq_inc_ / (float)sample_rate_;
    // resample_phase_ = 0.0f;
    min_uac_len_ever = 0xffffffff;
    max_uac_len_ever = 0;

    RCC_PLL3Cmd(DISABLE);
    RCC_I2S2CLKConfig(1);
    uint32_t prediv2 = ((RCC->CFGR2 & 0xf0) >> 4) + 1;
    // uint32_t pll3_in = HSE_VALUE / prediv2;
    // 在每个倍频系数下，寻找最接近的分频系数
    float min_frequency_error = 3840000.0f;
    uint32_t best_pll_mul = 0;
    uint32_t best_i2s_div = 0;
    for (uint32_t pll_mul = 0; pll_mul < NUM_PLL3_MUL; ++pll_mul) {
        float fi2s_div = (float)HSE_VALUE / prediv2 * kPLL3MulTable[pll_mul] / 256 / sample_rate;
        uint32_t i2s_div = (uint32_t)roundf(fi2s_div);
        if (i2s_div < MIN_I2S_PRESCALE || i2s_div > MAX_I2S_PRESCALE) {
            continue;
        }
        float frequency = (float)HSE_VALUE / prediv2 * kPLL3MulTable[pll_mul] / i2s_div / 256;
        if (fabsf(frequency - sample_rate) < min_frequency_error) {
            min_frequency_error = fabsf(frequency - sample_rate);
            best_pll_mul = pll_mul;
            best_i2s_div = i2s_div;
        }
    }
    RCC_PLL3Config(best_pll_mul << 12);
    I2S2_PrescaleConfig(best_i2s_div);
    RCC_PLL3Cmd(ENABLE);

    // wait for pll3
    while ((RCC->CTLR & (1 << 29))) {}
}

void Codec_MeasureSampleRateAndReportFeedback(void) {
    ++dma_frequency_meausure_counter_;
    if (dma_frequency_meausure_counter_ >= DMA_FREQUENCY_MEAURE_PERIOD) {
        // 10ms
        dma_frequency_meausure_counter_ = 0;
        uint32_t dma_bck = Codec_GetDMALen();
        mesured_dma_sample_rate_ = dma_bck * (1000 / (DMA_FREQUENCY_MEAURE_PERIOD / 8)) / 4;
    }

    ++feedback_report_counter_;
    if (feedback_report_counter_ >= FEEDBACK_REPORT_PERIOD) {
        // 1ms
        feedback_report_counter_ = 0;
        uint32_t uac_len = Codec_GetUACBufferLen();
        if (uac_len < UAC_BUFFER_LEN_DOWN_THRESHOLD) {
            USBUAC_WriteFeedback(mesured_dma_sample_rate_ + 1000);
        }
        else if (uac_len > UAC_BUFFER_LEN_UP_THRESHOLD) {
            USBUAC_WriteFeedback(mesured_dma_sample_rate_ - 1000);
        }
        else {
            USBUAC_WriteFeedback(mesured_dma_sample_rate_);
        }
    }
}
