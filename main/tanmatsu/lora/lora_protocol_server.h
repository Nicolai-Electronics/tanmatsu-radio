#pragma once

#include "esp_err.h"
#include "lora_protocol.h"

void lora_protocol_handle_packet(uint8_t* request_buffer, size_t request_length);
void start_lora_task();
