#pragma once

#include "esp_hosted_interface.h"

#define TANMATSU_IF_OFFSET 0x20

typedef enum {
    TANMATSU_FIRST_IF = TANMATSU_IF_OFFSET,
    TANMATSU_RFTEST_IF,
    TANMATSU_BADGELINK_IF,
    TANMATSU_802154_IF,
    TANMATSU_LORA_IF,
    TANMATSU_MAX_IF,
} tanmatsu_if_type_t;

static_assert(ESP_MAX_IF <= TANMATSU_IF_OFFSET, "Tanmatsu interfaces must not overlap with ESP interfaces");
