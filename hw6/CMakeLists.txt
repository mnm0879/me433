cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(i2c_screen
    i2c_screen.c
    ssd1306.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(i2c_screen pico_stdlib hardware_i2c hardware_adc)
target_include_directories(i2c_screen PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

# i2c_screen is the project name here
pico_enable_stdio_usb(i2c_screen 1) 
pico_enable_stdio_uart(i2c_screen 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(i2c_screen)