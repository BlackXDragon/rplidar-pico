cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rplidar_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_library(${PROJECT_NAME} src/rplidar.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pwm hardware_uart)

add_executable(example example.cpp)

target_link_libraries(example rplidar_pico)

pico_enable_stdio_usb(example 1)
pico_enable_stdio_uart(example 0)

pico_add_extra_outputs(example)
