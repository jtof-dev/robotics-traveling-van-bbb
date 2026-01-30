cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT on)
include(pico-sdk/pico_sdk_init.cmake)
project(bbba C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(bbb
	main.cpp
	PID_v1.cpp
)
pico_enable_stdio_usb(bbb 1)
pico_enable_stdio_uart(bbb 1)

pico_add_extra_outputs(bbb)
target_link_libraries(bbb pico_stdlib pico_multicore)
