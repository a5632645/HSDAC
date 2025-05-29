#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct _StereoSample {
    int32_t left;
    int32_t right;
} StereoSample_T;

enum I2C_Future_State {
    I2C_FUTURE_INIT_TX,
    I2C_FUTURE_ADDRESS_SENDED_TX,
    I2C_FUTURE_REG_SENDED_TX,
    I2C_FUTURE_REG_SENDED_RX,
    I2C_FUTURE_REG_SENDED_RX_RX,
    I2C_FUTURE_VAL_SENDED_TX,
    I2C_FUTURE_COMPLETED,

    I2C_FUTURE_INIT_RX,
    I2C_FUTURE_INIT_RX_RX,
    I2C_FUTURE_ADDRESS_SENDED_RX,
    I2C_FUTURE_ADDRESS_SENDED_RX_RX,
};

enum I2C_ERROR_CODE {
    I2C_EC_NO_ERROR,
    I2C_EC_NACK,
    I2C_EC_UNKOWN,
};

typedef struct _I2CFuture {
    uint8_t reg;
    uint8_t val;
    volatile enum I2C_Future_State state;
    volatile enum I2C_ERROR_CODE error_code;
} I2CFuture_T;

void            Codec_Init(void);
void            Codec_DeInit(void);
bool            Codec_IsTransferError(void);
StereoSample_T* Codec_GetBlockToFill(void);
uint32_t        Codec_GetBlockSize(void);
void            Codec_BlockFilled(void);
void            Codec_Write(I2CFuture_T* pstate);
void            Codec_Read(I2CFuture_T* pstate);
void            Codec_PollWrite(uint8_t reg, uint8_t val);
uint8_t         Codec_PollRead(uint8_t reg);

// clock sync
void     Codec_ClockSync_Reset(void);
uint32_t Codec_ClockSync_GetNumRead(void);
void     Codec_SetResampleRatio(float ratio);
void     Codec_Start(void);
void     Codec_Stop(void);
uint32_t Codec_GetDMASize(void);
uint32_t Codec_GetDMAHalfSize(void);
bool     Codec_IsDMAStart(void);
void     Codec_WriteUACBuffer(const uint8_t* ptr, uint32_t len);
uint32_t Codec_GetUACBufferLen(void);
void     Codec_CheckBuffer(void);

uint32_t Codec_GetDMALen(void);
