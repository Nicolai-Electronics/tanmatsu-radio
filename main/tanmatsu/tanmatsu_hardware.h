// Tanmatsu hardware definitions for the ESP32-C6 radio module
// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

// LoRa radio module pins
#define BSP_LORA_SCK   0
#define BSP_LORA_CS    1
#define BSP_LORA_MOSI  2
#define BSP_LORA_MISO  3
#define BSP_LORA_DIO1  4
#define BSP_LORA_BUSY  5
#define BSP_LORA_RESET 10
#define BSP_LORA_BUS   SPI2_HOST

// I2C bus pins
#define BSP_I2C_SDA 6
#define BSP_I2C_SCL 7

// Host interface pins
#define BSP_HOST_INT  8   // Output to ESP32-P4
#define BSP_HOST_BOOT 9   // Shared with USB enable (input)
#define BSP_HOST_TX   16  // UART connected to ESP32-P4 RX and internal expansion header
#define BSP_HOST_RX   17  // UART connected to ESP32-P4 TX and internal expansion header

// Infrared pins
#define BSP_IR_TX 15

// SDIO pins
#define BSP_SDIO_CMD 18
#define BSP_SDIO_CLK 19
#define BSP_SDIO_D0  20
#define BSP_SDIO_D1  21
#define BSP_SDIO_D2  22
#define BSP_SDIO_D3  23
