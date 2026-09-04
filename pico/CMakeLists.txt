# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w)

include(pico_sdk_import.cmake)

project(moving_trash_bin C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(moving_trash_bin
    main.c
    kinematics.c
    odometry_kalman.c
    motor_control.c
    communication.c
    encoder/encoder_pio.c
)

target_include_directories(moving_trash_bin PRIVATE ${CMAKE_CURRENT_LIST_DIR})

pico_generate_pio_header(moving_trash_bin ${CMAKE_CURRENT_LIST_DIR}/encoder/encoder.pio)

target_link_libraries(moving_trash_bin
    pico_stdlib
    hardware_pio
    hardware_pwm
    hardware_gpio
    hardware_clocks
)

pico_enable_stdio_usb(moving_trash_bin 1)
pico_enable_stdio_uart(moving_trash_bin 0)

pico_add_extra_outputs(moving_trash_bin)
