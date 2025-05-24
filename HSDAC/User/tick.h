#pragma once
#include <stdint.h>

void     Tick_Init(void);
uint32_t Tick_GetTick(void);
void     Tick_DelayMs(uint32_t ms);