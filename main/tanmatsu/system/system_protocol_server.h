#pragma once

#include <stdint.h>
#include "esp_err.h"

#define SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH 16

typedef enum {
    SYSTEM_PROTOCOL_TYPE_ACK             = 0x00,
    SYSTEM_PROTOCOL_TYPE_NACK            = 0x01,
    SYSTEM_PROTOCOL_TYPE_GET_INFORMATION = 0x02,
    SYSTEM_PROTOCOL_TYPE_NVS_LIST        = 0x10,
    SYSTEM_PROTOCOL_TYPE_NVS_READ        = 0x11,
    SYSTEM_PROTOCOL_TYPE_NVS_WRITE       = 0x12,
    SYSTEM_PROTOCOL_TYPE_NVS_DELETE      = 0x13,
    SYSTEM_PROTOCOL_TYPE_APPFS_LIST      = 0x20,
    SYSTEM_PROTOCOL_TYPE_APPFS_READ      = 0x21,
    SYSTEM_PROTOCOL_TYPE_APPFS_WRITE     = 0x22,
    SYSTEM_PROTOCOL_TYPE_APPFS_DELETE    = 0x23,
    SYSTEM_PROTOCOL_TYPE_APPFS_CHECKSUM  = 0x24,
    SYSTEM_PROTOCOL_TYPE_APPFS_GET_USAGE = 0x25,
    SYSTEM_PROTOCOL_TYPE_APPFS_BOOT      = 0x26,
    SYSTEM_PROTOCOL_TYPE_FS_LIST         = 0x30,
    SYSTEM_PROTOCOL_TYPE_FS_READ         = 0x31,
    SYSTEM_PROTOCOL_TYPE_FS_WRITE        = 0x32,
    SYSTEM_PROTOCOL_TYPE_FS_DELETE       = 0x33,
    SYSTEM_PROTOCOL_TYPE_FS_CHECKSUM     = 0x34,
    SYSTEM_PROTOCOL_TYPE_FS_STAT         = 0x35,
    SYSTEM_PROTOCOL_TYPE_FS_MKDIR        = 0x36,
    SYSTEM_PROTOCOL_TYPE_FS_RMDIR        = 0x37,
    SYSTEM_PROTOCOL_TYPE_FS_GET_USAGE    = 0x38,
    SYSTEM_PROTOCOL_TYPE_FS_COPY         = 0x39,
    SYSTEM_PROTOCOL_TYPE_FS_RENAME       = 0x3A,
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

// NVS

typedef enum {
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT8  = 0,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT8   = 1,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT16 = 2,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT16  = 3,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT32 = 4,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT32  = 5,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_UINT64 = 6,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_INT64  = 7,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_STRING = 8,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_BLOB   = 9,
    SYSTEM_PROTOCOL_NVS_VALUE_TYPE_ANY    = 10,
} system_protocol_nvs_value_type_t;

typedef struct {
    char                             key[SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH];
    system_protocol_nvs_value_type_t type;
} __attribute__((packed)) system_protocol_nvs_entry_t;

typedef struct {
    char                             namespace_name[SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH];
    char                             key[SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH];
    system_protocol_nvs_value_type_t type;
    uint32_t                         offset;
} __attribute__((packed)) system_protocol_nvs_list_request_t;

typedef struct {
    uint32_t                    count;
    system_protocol_nvs_entry_t entries[0];
} __attribute__((packed)) system_protocol_nvs_list_response_t;

typedef struct {
    char                        namespace_name[SYSTEM_PROTOCOL_NVS_MAX_KEY_LENGTH];
    system_protocol_nvs_entry_t entry;
} __attribute__((packed)) system_protocol_nvs_location_t;

typedef struct {
    system_protocol_nvs_location_t location;
    uint32_t                       value_length;
    union {
        uint8_t  value_u8;
        int8_t   value_i8;
        uint16_t value_u16;
        int16_t  value_i16;
        uint32_t value_u32;
        int32_t  value_i32;
        uint64_t value_u64;
        int64_t  value_i64;
        char     value_string[0];
        uint8_t  value_blob[0];
    };
} __attribute__((packed)) system_protocol_nvs_value_t;

esp_err_t system_protocol_initialize(void);
