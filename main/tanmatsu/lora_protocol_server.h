#pragma once

#include "esp_err.h"
#include "lora_protocol.h"

esp_err_t lora_protocol_handle_packet(uint8_t* data, size_t length);
