cmake_minimum_required(VERSION 3.5)

if(DEVICE STREQUAL "")
	set(DEVICE "$ENV{TARGET}")
endif()

if(DEVICE STREQUAL "")
	message(FATAL_ERROR "DEVICE not set")
endif()

if(DEVICE STREQUAL "tanmatsu")
	set(SDKCONFIG_DEFAULTS "sdkconfigs/general;sdkconfigs/tanmatsu")
	set(IDF_TARGET esp32p4)
	set(SDKCONFIG "sdkconfig_tanmatsu")
else()
	message(FATAL_ERROR "Unknown target device ${DEVICE}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(APPEND EXCLUDE_COMPONENTS lwip esp_netif)
set(COMPONENTS esp_timer esptool_py bootloader main nvs_flash esp_rom esp_wifi protocomm )
project(${DEVICE}-radio)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
