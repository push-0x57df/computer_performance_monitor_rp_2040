cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(computer_performance_monitor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(computer_performance_monitor
    rtos_queue.c main.c rtos_task.c HarmonyOS_2bit.c
)

# enable usb output, disable uart output
pico_enable_stdio_usb(computer_performance_monitor 1)
pico_enable_stdio_uart(computer_performance_monitor 0)

pico_generate_pio_header(computer_performance_monitor ${CMAKE_CURRENT_LIST_DIR}/i8080.pio)

target_include_directories(computer_performance_monitor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR})

# Specify path to own LVGL config header
set(LV_CONF_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lv_conf.h
    CACHE STRING "" FORCE)

add_subdirectory(lvgl-8.3.10)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(computer_performance_monitor)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(computer_performance_monitor pico_stdlib hardware_pio hardware_dma FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 lvgl::lvgl lvgl::demos)