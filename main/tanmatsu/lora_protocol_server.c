#include "lora_protocol_server.h"
#include "esp_err.h"
#include "lora_protocol.h"

esp_err_t lora_protocol_handle_packet(uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");

    lora_protocol_packet_t* packet = (lora_protocol_packet_t*)data;

    switch (packet->command) {
        case LORA_PROTOCOL_COMMAND_GET_MODE:
            // Handle GET_MODE command
            printf("Handling GET_MODE command\r\n");
            break;
        case LORA_PROTOCOL_COMMAND_SET_MODE:
            // Handle SET_MODE command
            printf("Handling SET_MODE command\r\n");
            break;
        case LORA_PROTOCOL_COMMAND_GET_CONFIG:
            // Handle GET_CONFIG command
            printf("Handling GET_CONFIG command\r\n");
            break;
        case LORA_PROTOCOL_COMMAND_SET_CONFIG:
            // Handle SET_CONFIG command
            printf("Handling SET_CONFIG command\r\n");
            break;
        case LORA_PROTOCOL_COMMAND_GET_STATUS:
            // Handle GET_STATUS command
            printf("Handling GET_STATUS command\r\n");
            break;
        case LORA_PROTOCOL_COMMAND_PACKET_RX:
            // Handle PACKET_RX command
            printf("Handling PACKET_RX command\r\n");
            break;
        case LORA_PROTOCOL_COMMAND_PACKET_TX:
            // Handle PACKET_TX command
            printf("Handling PACKET_TX command\r\n");
            break;
        default:
            printf("Unknown command: %d\r\n", packet->command);
            break;
    }

    return ESP_OK;
}