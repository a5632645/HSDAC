#pragma once
#include <stdint.h>
#include <stdbool.h>

#define HID_QUEUE_SIZE 64
#define HID_QUEUE_SIZE_MASK (HID_QUEUE_SIZE - 1)

struct HID_Event {
    uint8_t type;
    uint8_t reg;
    uint8_t val;
    uint8_t _;
};

struct HID_Queue {
    uint32_t wpos_;
    uint32_t rpos_;
    uint32_t num_;
    struct HID_Event events_[HID_QUEUE_SIZE];
};
extern struct HID_Queue g_hid_queue;

void HID_Queue_Read(struct HID_Queue* queue, uint32_t* num);
void HID_Queue_Finish(struct HID_Queue* queue, uint32_t num);
struct HID_Event* HID_Queue_NextDMAItem_IRQ(struct HID_Queue* queue);
struct HID_Event* HID_Queue_GetItem(struct HID_Queue* queue, uint32_t offset);