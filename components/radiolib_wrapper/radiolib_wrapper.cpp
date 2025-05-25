#include "radiolib_wrapper.h"
#include "RadioLib.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "radiolib_hal.hpp"

static const char* TAG = "radiolib wrapper";

RadioLibEspHal* hal   = NULL;
SX1262          radio = NULL;

extern "C" void radiolib_initialize(spi_device_handle_t spi_handle, gpio_num_t reset_pin, gpio_num_t busy_pin,
                                    gpio_num_t dio0_pin, gpio_num_t dio1_pin, gpio_num_t dio2_pin) {
    hal   = new RadioLibEspHal(spi_handle, reset_pin, dio0_pin, dio1_pin, dio2_pin);
    radio = new Module(hal, RADIOLIB_NC, dio1_pin, reset_pin, busy_pin);
    ESP_LOGI(TAG, "Initializing ... ");

    int state = radio.begin(869.525f, 250, 11, 8, 0x2b, 10, 16, 1.8f, false);

    if (state != RADIOLIB_ERR_NONE) {
        if (state == RADIOLIB_ERR_SPI_CMD_FAILED) {
            state = radio.begin(869.525f, 250, 11, 8, 0x2b, 10, 16, 0.0f, false);
            if (state != RADIOLIB_ERR_NONE) {
                ESP_LOGE(TAG, "failed, code %d\n", state);
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            } else {
                ESP_LOGI(TAG, "Initialized radio with XTAL!\n");
            }
        } else {
            ESP_LOGE(TAG, "failed, code %d\n", state);
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    } else {
        ESP_LOGI(TAG, "Initialized radio with TCXO!\n");
    }

    radio.setDio2AsRfSwitch(true);

    ESP_LOGI(TAG, "success!\n");
}

extern "C" void radiolib_test(void) {

    /*int state = radio.transmitDirect(869.525f);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to transmit carrier, code %d\n", state);
    } else {
        ESP_LOGI(TAG, "Transmitting carrier!\n");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }*/

    printf("Testing!\r\n");

    while (1) {
        // send a packet
        ESP_LOGI(TAG, "[SX1262] Transmitting packet ... ");
        int state = radio.transmit("Hello World!");
        if (state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            ESP_LOGI(TAG, "success!");

        } else {
            ESP_LOGI(TAG, "failed, code %d\n", state);
        }
        ESP_LOGI(TAG, "[SX1262] Transmission finished");

        for (uint8_t i = 0; i < 10; i++) {
            uint8_t data[32] = {0};
            state            = radio.receive(data, sizeof(data));

            if (state == RADIOLIB_ERR_NONE) {
                // the packet was successfully received
                ESP_LOGI(TAG, "[SX1262] Received packet");
                for (size_t i = 0; i < sizeof(data); i++) {
                    printf("%02x ", data[i]);
                }
                printf("\r\n");
            } else {
                ESP_LOGI(TAG, "[SX1262] Failed to receive packet, code %d", state);
            }
        }
    }
}
