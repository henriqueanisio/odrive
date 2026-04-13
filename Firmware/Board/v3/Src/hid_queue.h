#ifndef HID_QUEUE_H
#define HID_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define HID_QUEUE_SIZE 16
#define HID_MAX_REPORT_SIZE 64

typedef struct {
    uint8_t data[HID_MAX_REPORT_SIZE];
    uint16_t len;
} HidReport_t;

void hid_queue_init(void);
bool hid_queue_push(uint8_t *data, uint16_t len);
bool hid_queue_push_priority(uint8_t *data, uint16_t len);
void hid_queue_process(void);

#endif