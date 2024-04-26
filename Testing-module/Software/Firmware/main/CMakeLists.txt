cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(firmware_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(COMPILE_FLAGS "-o2 -Wpedantic")

pico_sdk_init()

add_executable(main
        main.cpp
        #${PROJECT_SOURCE_DIR}/../modules/can2040/src/can2040.c
        ${PROJECT_SOURCE_DIR}/../components/adc/adc.cpp
        ${PROJECT_SOURCE_DIR}/../components/can/can.cpp
        ${PROJECT_SOURCE_DIR}/../components/signal-generation/random-signal.cpp
        ${PROJECT_SOURCE_DIR}/../components/signal-generation/sawtooth-signal.cpp
        ${PROJECT_SOURCE_DIR}/../components/signal-generation/rpm.cpp
        ${PROJECT_SOURCE_DIR}/../components/heartbeat/heartbeat.cpp
        ${PROJECT_SOURCE_DIR}/../components/pico-mcp2515/include/mcp2515/mcp2515.cpp
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(main pico_stdlib hardware_adc hardware_spi)

target_include_directories(main PRIVATE $ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include)
target_include_directories(main PRIVATE $ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# enable pico to send data via uart
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)