#include "codec.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_spi.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_i2c.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define I2S_DMA_BLOCK_SIZE 1024
#define I2S_DMA_BUFFER_SIZE (I2S_DMA_BLOCK_SIZE * 2)
static StereoSample_T i2s_dma_buffer_[I2S_DMA_BUFFER_SIZE] = {0};
static volatile bool transfer_error_ = false;
static volatile StereoSample_T* block_to_fill = NULL;

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
    I2S_InitStructure.I2S_AudioFreq = I2S_AudioFreq_192k;
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
        block_to_fill = &i2s_dma_buffer_[I2S_DMA_BLOCK_SIZE];
    }
    if (DMA_GetFlagStatus(DMA1_FLAG_HT5) == SET) {
        DMA_ClearFlag(DMA1_FLAG_HT5);
        block_to_fill = &i2s_dma_buffer_[0];
    }
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
    int32_t ret;
    int16_t* a = &x;
    int16_t* b = &ret;
    b[0] = a[1];
    b[1] = a[0];
    return ret;
}

void Codec_Init(void) {
    I2S2_Init();
    I2C2_Init();

    for (uint32_t i = 0; i < I2S_DMA_BUFFER_SIZE; ++i) {
        float e = (float)i / (float)(I2S_DMA_BUFFER_SIZE) * 2.0f * M_PI;
        float sin = sinf(e);
        float cosine = cosf(e);
        const int32_t eee = (16384 << 16) + 1;
        i2s_dma_buffer_[i].left = Swap16(eee * sin);
        i2s_dma_buffer_[i].right = Swap16(eee * cosine);
    }

    DMA_Tx_Init(DMA1_Channel5, (u32)&SPI2->DATAR, (u32)&i2s_dma_buffer_, I2S_DMA_BUFFER_SIZE * 4);
    DMA_Cmd(DMA1_Channel5, ENABLE);

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

StereoSample_T* Codec_GetBlockToFill(void) {
    return block_to_fill;
}

uint32_t Codec_GetBlockSize(void) {
    return I2S_DMA_BLOCK_SIZE;
}

void Codec_BlockFilled(void) {
    block_to_fill = NULL;
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