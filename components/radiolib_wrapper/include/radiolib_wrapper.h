#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

void radiolib_initialize(spi_device_handle_t spi_handle, gpio_num_t reset_pin, gpio_num_t busy_pin, gpio_num_t dio0_pin,
                         gpio_num_t dio1_pin, gpio_num_t dio2_pin);
void radiolib_test(void);

#ifdef __cplusplus
}
#endif
