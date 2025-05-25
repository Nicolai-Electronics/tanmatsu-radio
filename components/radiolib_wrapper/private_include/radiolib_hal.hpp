#pragma once

#include "RadioLib.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_hal.h"
#include "rom/gpio.h"
#include "soc/rtc.h"

#define LOW     (0x0)
#define HIGH    (0x1)
#define INPUT   (0x01)
#define OUTPUT  (0x03)
#define RISING  (0x01)
#define FALLING (0x02)
#define NOP()   asm volatile("nop")

#define MATRIX_DETACH_OUT_SIG    (0x100)
#define MATRIX_DETACH_IN_LOW_PIN (0x30)

#define ClkRegToFreq(reg) (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))

class RadioLibEspHal : public RadioLibHal {
   public:
    RadioLibEspHal(spi_device_handle_t spi_handle, uint32_t reset_pin, uint32_t dio0_pin, uint32_t dio1_pin,
                   uint32_t dio2_pin);

    void          init() override;
    void          term() override;
    void          pinMode(uint32_t pin, uint32_t mode) override;
    void          digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t      digitalRead(uint32_t pin) override;
    void          attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    void          detachInterrupt(uint32_t interruptNum) override;
    void          delay(unsigned long ms) override;
    void          delayMicroseconds(unsigned long us) override;
    unsigned long millis() override;
    unsigned long micros() override;
    long          pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override;
    void          spiBegin();
    void          spiBeginTransaction();
    uint8_t       spiTransferByte(uint8_t b);
    void          spiTransfer(uint8_t* out, size_t len, uint8_t* in);
    void          spiEndTransaction();
    void          spiEnd();

   private:
    spi_device_handle_t m_spi_handle;
    uint32_t            m_reset_pin;
    uint32_t            m_dio0_pin;
    uint32_t            m_dio1_pin;
    uint32_t            m_dio2_pin;
};
