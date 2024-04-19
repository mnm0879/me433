cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(more_io
    more_io.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(more_io pico_stdlib hardware_i2c)

# more_io is the project name here
pico_enable_stdio_usb(more_io 1) 
pico_enable_stdio_uart(more_io 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(more_io)