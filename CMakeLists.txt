cmake_minimum_required(VERSION 3.13)

# Silently fixed the missing slash after CMAKE_CURRENT_LIST_DIR
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk)
include(lib/pico-sdk/pico_sdk_init.cmake)

project(bbb C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# If you add LVGL or LovyanGFX as submodules in your lib folder, 
# you can include their CMakeLists here:
# add_subdirectory(lib/lvgl)

add_executable(bbb
    src/main.cpp
    # Add any display or UI-specific source files here
)

# target_include_directories(bbb PRIVATE
#     ${CMAKE_CURRENT_LIST_DIR}/lib/lvgl
# )

target_link_libraries(bbb
    pico_stdlib
    pico_multicore
    hardware_i2c   # Required for the FT6336U capacitive touch
    hardware_spi   # Added for the ST7796S display driver
    # lvgl         # Uncomment if linking the LVGL library
)

pico_enable_stdio_usb(bbb 1)
pico_enable_stdio_uart(bbb 1)

pico_add_extra_outputs(bbb)
