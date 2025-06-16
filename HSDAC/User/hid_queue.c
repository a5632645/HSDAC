#include "hid_queue.h"
#include "ch32v30x.h"
#include "ch32v30x_usb.h"
#include <stdatomic.h>

struct HID_Queue g_hid_queue = {
    .wpos_ = 0,
    .rpos_ = 0,
    .num_ = 0,
};

void HID_Queue_Read(struct HID_Queue* queue, uint32_t* num) {
    *num = atomic_load(&queue->num_);
}

void HID_Queue_Finish(struct HID_Queue* queue, uint32_t num) {
    if (num > 0) {
        queue->rpos_ = (queue->rpos_ + num) & HID_QUEUE_SIZE_MASK;
        if ((USBHSD->UEP4_RX_CTRL & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_NAK) {
            USBHSD->UEP4_RX_DMA = (uint32_t)&queue->events_[queue->wpos_];
            USBHSD->UEP4_RX_CTRL = (USBHSD->UEP4_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
        }
        atomic_fetch_sub(&queue->num_, num);
    }
}

struct HID_Event* HID_Queue_NextDMAItem_IRQ(struct HID_Queue* queue) {
    queue->wpos_ = (queue->wpos_ + 1) & HID_QUEUE_SIZE_MASK;
    if (queue->num_ == HID_QUEUE_SIZE) {
        // full
        return NULL;
    }
    else {
        struct HID_Event* e = &queue->events_[queue->wpos_];
        return e;
    }
}

struct HID_Event* HID_Queue_GetItem(struct HID_Queue* queue, uint32_t offset) {
    return &queue->events_[(queue->rpos_ + offset) & HID_QUEUE_SIZE_MASK];
}