cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(uart
    uart.c
    ssd1306.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(uart pico_stdlib hardware_i2c hardware_irq hardware_uart)
target_include_directories(uart PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

# uart is the project name here
pico_enable_stdio_usb(uart 1) 
pico_enable_stdio_uart(uart 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(uart)