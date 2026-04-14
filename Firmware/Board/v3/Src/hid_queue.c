#include "hid_queue.h"
#include "usbd_hid.h"
#include "usb_device.h"
#include <string.h>

static HidReport_t queue[HID_QUEUE_SIZE];
static volatile uint8_t head = 0;
static volatile uint8_t tail = 0;

static inline bool is_full(void)
{
    return ((head + 1) % HID_QUEUE_SIZE) == tail;
}

static inline bool is_empty(void)
{
    return head == tail;
}

void hid_queue_init(void)
{
    head = tail = 0;
}

bool hid_queue_push(uint8_t *data, uint16_t len)
{
    uint32_t next = (head + 1) % HID_QUEUE_SIZE;
    if (next == tail) return false; // Fila cheia, descarta (melhor que corromper)

    memcpy(queue[head].data, data, len);
    queue[head].len = len;
    head = next;

    return true;
}

void hid_queue_process(void)
{
    if (is_empty()) return;

    USBD_HID_HandleTypeDef *hhid =
        (USBD_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

    if (!hhid) return;

    if (hhid->state != HID_IDLE) return;

    USBD_HID_SendReport(&hUsbDeviceFS,
        queue[tail].data,
        queue[tail].len);

    tail = (tail + 1) % HID_QUEUE_SIZE;
}