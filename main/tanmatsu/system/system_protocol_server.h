#pragma once

#include "esp_err.h"

typedef enum {
    SYSTEM_PROTOCOL_TYPE_ACK             = 0x00,
    SYSTEM_PROTOCOL_TYPE_NACK            = 0x01,
    SYSTEM_PROTOCOL_TYPE_GET_INFORMATION = 0x02,
} system_protocol_packet_type_t;

typedef struct {
    uint32_t sequence_number;
    uint32_t type;  // system_protocol_packet_type_t
} __attribute__((packed)) system_protocol_header_t;

typedef struct {
    char firmware_name[32];
    char firmware_version[32];
    char firmware_build_date[16];
    char firmware_build_time[16];
    char firmware_idf_version[32];
    char firmware_sha256[32];
} __attribute__((packed)) system_protocol_information_t;

esp_err_t system_protocol_initialize(void);
