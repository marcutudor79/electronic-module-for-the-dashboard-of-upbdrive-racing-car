cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(firmware_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(COMPILE_FLAGS "-o2 -Wpedantic")
set(CMAKE_GENERATOR "MinGW Makefiles")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

pico_sdk_init()

add_executable(main
        main.cpp
        ${PROJECT_SOURCE_DIR}/../components/can2040/src/can2040.c
        ${PROJECT_SOURCE_DIR}/../components/adc/adc.cpp
        ${PROJECT_SOURCE_DIR}/../components/can/can.cpp
        ${PROJECT_SOURCE_DIR}/../components/signal-generation/sawtooth-signal.cpp
        ${PROJECT_SOURCE_DIR}/../components/heartbeat/heartbeat.cpp
        ${PROJECT_SOURCE_DIR}/../components/pico-mcp2515/include/mcp2515/mcp2515.cpp
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(main pico_stdlib hardware_adc hardware_spi)

# Include header directories
target_include_directories(main PRIVATE
    $ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include
    $ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/../components/adc/inc
    ${CMAKE_SOURCE_DIR}/../components/can/inc
    ${CMAKE_SOURCE_DIR}/../components/common/inc
    ${CMAKE_SOURCE_DIR}/../components/heartbeat/inc
    ${CMAKE_SOURCE_DIR}/../components/signal-generation/inc
    ${CMAKE_SOURCE_DIR}/../components/pico-mcp2515/include/mcp2515
    ${CMAKE_SOURCE_DIR}/../components/can2040/src
)


# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# enable pico to send data via uart
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)