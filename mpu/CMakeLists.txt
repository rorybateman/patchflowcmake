cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
#includes the necesserey directories withinn the sdk such as spi.h
include_directories(${PICO_SDK_INCLUDE_DIRS})

project(eulerAngleExample)

# not quite sure what this is doing tbh
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()



add_executable(eulerAngleExample eulerAngleExample.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(eulerAngleExample PRIVATE pico_stdlib pico_mpu_9250)

pico_enable_stdio_usb(eulerAngleExample 1)
pico_enable_stdio_uart(eulerAngleExample 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(eulerAngleExample)

