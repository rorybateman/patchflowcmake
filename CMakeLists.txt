cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(hello_world)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
if (TARGET tinyusb_device)
    add_executable(hello_world
            hello_world.c
            )

    # pull in common dependencies
    target_link_libraries(hello_world pico_stdlib)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(hello_world 1)
    pico_enable_stdio_uart(hello_world 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(hello_world)

    # add url via pico_set_program_url
    example_auto_set_url(hello_world)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_world because TinyUSB submodule is not initialized in the SDK")
endif()
