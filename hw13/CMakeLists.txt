cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(mouse
    mouse.c
)

set(FAMILY rp2040)
set(BOARD pico_sdk)
set(TINYUSB_FAMILY_PROJECT_NAME_PREFIX "tinyusb_dev_")

# Add pico_stdlib library which aggregates commonly used features
target_sources(mouse PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/mouse.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        )
target_include_directories(mouse PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(mouse PUBLIC pico_stdlib pico_unique_id tinyusb_device tinyusb_board hardware_i2c)

# more_io is the project name here
pico_enable_stdio_usb(mouse 1) 
pico_enable_stdio_uart(mouse 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(mouse)