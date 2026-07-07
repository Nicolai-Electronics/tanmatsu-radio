#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    INFRARED_PROTOCOL_TYPE_ACK             = 0x00,
    INFRARED_PROTOCOL_TYPE_NACK            = 0x01,
    INFRARED_PROTOCOL_TYPE_GET_INFORMATION = 0x02,
    INFRARED_PROTOCOL_TYPE_SEND_NEC        = 0x03,
} infrared_protocol_packet_type_t;

typedef struct {
    uint32_t sequence_number;
    uint32_t type;  // infrared_protocol_packet_type_t
} __attribute__((packed)) infrared_protocol_header_t;

typedef struct {
    bool available;
} __attribute__((packed)) infrared_protocol_information_t;

esp_err_t infrared_protocol_initialize(void);
esp_err_t infrared_protocol_reconfigure(void);
