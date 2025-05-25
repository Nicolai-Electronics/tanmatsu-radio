#include "radiolib_hal.hpp"
#include "RadioLib.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_hal.h"

RadioLibEspHal::RadioLibEspHal(spi_device_handle_t spi_handle, uint32_t reset_pin, uint32_t dio0_pin, uint32_t dio1_pin,
                               uint32_t dio2_pin)
    : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {
    m_spi_handle = spi_handle;
    m_reset_pin  = reset_pin;
    m_dio0_pin   = dio0_pin;
    m_dio1_pin   = dio1_pin;
    m_dio2_pin   = dio2_pin;
}

void RadioLibEspHal::init() {
    spiBegin();
}

void RadioLibEspHal::term() {
    spiEnd();
}

void RadioLibEspHal::pinMode(uint32_t pin, uint32_t mode) {
    if (pin == RADIOLIB_NC) {
        return;
    }

    gpio_hal_context_t gpiohal;
    gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);

    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = (gpio_mode_t)mode,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = (gpio_int_type_t)gpiohal.dev->pin[pin].int_type,
    };
    gpio_config(&conf);
}

void RadioLibEspHal::digitalWrite(uint32_t pin, uint32_t value) {
    if (pin == RADIOLIB_NC) {
        return;
    }

    gpio_set_level((gpio_num_t)pin, value);
}

uint32_t RadioLibEspHal::digitalRead(uint32_t pin) {
    if (pin == RADIOLIB_NC) {
        return (0);
    }

    return (gpio_get_level((gpio_num_t)pin));
}

void RadioLibEspHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
    if (interruptNum == RADIOLIB_NC) {
        return;
    }

    gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);
    gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

    // this uses function typecasting, which is not defined when the functions have different signatures
    // untested and might not work
    gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void*))interruptCb, NULL);
}

void RadioLibEspHal::detachInterrupt(uint32_t interruptNum) {
    if (interruptNum == RADIOLIB_NC) {
        return;
    }

    gpio_isr_handler_remove((gpio_num_t)interruptNum);
    gpio_wakeup_disable((gpio_num_t)interruptNum);
    gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
}

void RadioLibEspHal::delay(unsigned long ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void RadioLibEspHal::delayMicroseconds(unsigned long us) {
    uint64_t m = (uint64_t)esp_timer_get_time();
    if (us) {
        uint64_t e = (m + us);
        if (m > e) {  // overflow
            while ((uint64_t)esp_timer_get_time() > e) {
                NOP();
            }
        }
        while ((uint64_t)esp_timer_get_time() < e) {
            NOP();
        }
    }
}

unsigned long RadioLibEspHal::millis() {
    return ((unsigned long)(esp_timer_get_time() / 1000ULL));
}

unsigned long RadioLibEspHal::micros() {
    return ((unsigned long)(esp_timer_get_time()));
}

long RadioLibEspHal::pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) {
    if (pin == RADIOLIB_NC) {
        return (0);
    }

    this->pinMode(pin, INPUT);
    uint32_t start   = this->micros();
    uint32_t curtick = this->micros();

    while (this->digitalRead(pin) == state) {
        if ((this->micros() - curtick) > timeout) {
            return (0);
        }
    }

    return (this->micros() - start);
}

void RadioLibEspHal::spiBegin() {
}

void RadioLibEspHal::spiBeginTransaction() {
    // not needed - in ESP32 Arduino core, this function
    // repeats clock div, mode and bit order configuration
}

uint8_t RadioLibEspHal::spiTransferByte(uint8_t b) {
    /*this->spi->mosi_dlen.usr_mosi_dbitlen = 7;
    this->spi->miso_dlen.usr_miso_dbitlen = 7;
    this->spi->data_buf[0]                = b;
    this->spi->cmd.usr                    = 1;
    while (this->spi->cmd.usr);
    return (this->spi->data_buf[0] & 0xFF);*/
    printf("stub: RadioLibEspHal::spiTransferByte\r\n");
    return 0;
}

void RadioLibEspHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
    esp_err_t res;

    spi_transaction_t t = {0};
    t.length            = len * 8;
    t.tx_buffer         = out;
    t.rx_buffer         = in;
    res                 = spi_device_transmit(m_spi_handle, &t);
    if (res != ESP_OK) {
        ESP_LOGE("RadioLibEspHal", "SPI transfer failed: %s", esp_err_to_name(res));
    }
}

void RadioLibEspHal::spiEndTransaction() {
}

void RadioLibEspHal::spiEnd() {
}
