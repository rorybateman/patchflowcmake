cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(mpu)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

#rest of project
add_executable(mpu
        mpu6050_spi.c
        )

# pull in common dependencies and additional spi hardware support
target_link_libraries(mpu9250_spi pico_stdlib hardware_spi)

# enable usb output, disable uart output
pico_enable_stdio_usb(mpu 1)
pico_enable_stdio_uart(mpu 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu)

# add url via pico_set_program_url
#example_auto_set_url(mpu)
