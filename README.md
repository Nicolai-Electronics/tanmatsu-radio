# Tanmatsu radio firmware

Firmware for the ESP32-C6 radio module on the Tanmatsu.

## Source

Based on [esp-hosted-mcu](https://github.com/espressif/esp-hosted-mcu/).

Upstream version: v2.12.3

## How to update

Note that these instructions might not work on future releases of ESP-HOSTED, please adjust where needed.

- Download the latest release archive from the [Component Registry](https://components.espressif.com/components/espressif/esp_hosted/versions/2.12.3/readme) by clicking "download archive".
- Delete all files from the `main` folder, except for the `Tanmatsu` folder.
- Copy all files from the `slave/main` folder into the `main` folder.
- Open `main/CMakeLists.txt` with an editor and add the snippet below above `register_component()`:

```
# Requires
set(COMPONENT_REQUIRES esp_timer esptool_py bootloader main nvs_flash esp_rom esp_wifi wpa_supplicant protocomm protobuf-c driver app_update esp_driver_gpio esp_driver_spi)

# Tanmatsu
set(tanmatsu_dir "tanmatsu")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/tanmatsu_main.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/lora/lora_protocol_server.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/badgelink/badgelink_protocol_server.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/echo/echo_protocol_server.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/infrared/ir_nec_encoder.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/infrared/ir_protocol_server.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/system/system_protocol_server.c")
list(APPEND COMPONENT_SRCS "${tanmatsu_dir}/ieee802154/ieee802154_protocol_server.c")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}/lora")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}/badgelink")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}/echo")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}/infrared")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}/system")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "${tanmatsu_dir}/ieee802154")
```

## Authors

Copyright 2026 Nicolai Electronics

Copyright 2021-2026 Espressif Systems (Shanghai) CO LTD

License: [Apache-2.0](LICENSE)
