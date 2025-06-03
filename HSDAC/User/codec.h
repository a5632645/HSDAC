#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct _StereoSample {
    int32_t left;
    int32_t right;
} StereoSample_T;

void            Codec_Init(void);
void            Codec_DeInit(void);
bool            Codec_IsTransferError(void);
StereoSample_T* Codec_GetBlockToFill(void);
uint32_t        Codec_GetBlockSize(void);
void            Codec_BlockFilled(void);
void            Codec_PollWrite(uint8_t reg, uint8_t val);
uint8_t         Codec_PollRead(uint8_t reg);

// clock sync
void     Codec_Start(void);
void     Codec_Stop(void);
uint32_t Codec_GetDMASize(void);
uint32_t Codec_GetDMAHalfSize(void);
bool     Codec_IsDMAStart(void);
void     Codec_WriteUACBuffer(const uint8_t* ptr, uint32_t len);
uint32_t Codec_GetUACBufferLen(void);
void     Codec_CheckBuffer(void);

uint32_t Codec_GetDMALen(void);
void     Codec_SetSampleRate(uint32_t sample_rate);
void     Codec_MeasureSampleRateAndReportFeedback(void);

enum eCodecChannel {
    eCodecChannel_Left,
    eCodecChannel_Right,
};

void     Codec_CheckVolumeEvent(void);
void     Codec_SetVolume(enum eCodecChannel channel, uint8_t vol);