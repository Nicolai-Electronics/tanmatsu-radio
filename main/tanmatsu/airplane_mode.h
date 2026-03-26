// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Initialize airplane mode: load state from NVS, register callback, and apply.
// Must be called after nvs_flash_init() and esp_hosted_coprocessor_init().
esp_err_t airplane_mode_init(void);

// Returns true if airplane mode is currently enabled.
bool airplane_mode_is_enabled(void);
