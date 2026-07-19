// WHY2025 badge hardware definitions for the ESP32-C6 radio module
// SPDX-FileCopyrightText: 2026 Senna Hijlkema
// SPDX-License-Identifier: MIT

#pragma once

// LoRa radio module pins
#define BSP_LORA_SCK   6
#define BSP_LORA_CS    4
#define BSP_LORA_MOSI  7
#define BSP_LORA_MISO  2
#define BSP_LORA_DIO1  5
#define BSP_LORA_BUSY  11
#define BSP_LORA_RESET 1
#define BSP_LORA_BUS   SPI2_HOST

// Host interface pins
#define BSP_HOST_INT  0   // Output to ESP32-P4
#define BSP_HOST_BOOT 9   // Shared with USB enable (input)
#define BSP_HOST_TX   16  // UART connected to ESP32-P4 RX and internal expansion header
#define BSP_HOST_RX   17  // UART connected to ESP32-P4 TX and internal expansion header

// Infrared pins
#define BSP_IR_TX 11

// WHYSpecific pins
#define BSP_KEYBOARD_BL 10
#define BSP_DISPLAY_BL 15

// SDIO pins
#define BSP_SDIO_CMD 18
#define BSP_SDIO_CLK 19
#define BSP_SDIO_D0  20
#define BSP_SDIO_D1  21
#define BSP_SDIO_D2  22
#define BSP_SDIO_D3  23

// WHY Specific things for backlight