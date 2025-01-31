cmake_minimum_required(VERSION 3.13)

# Add executable. Default name is the project name, version 0.1
add_executable(midi-keyboard)

# pico_enable_stdio_usb()
target_sources(midi-keyboard PUBLIC
${CMAKE_CURRENT_LIST_DIR}/main.c
${CMAKE_CURRENT_LIST_DIR}/circular.c
${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        )

pico_set_program_name(midi-keyboard "midi-keyboard")
pico_set_program_version(midi-keyboard "0.1")
#pico_enable_stdio_usb(midi-keyboard)

# Generate PIO header
pico_generate_pio_header(midi-keyboard ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
# add_subdirectory(midi-keyboard)
# add_subdirectory(tinyusb)
# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(midi-keyboard 1)
pico_enable_stdio_usb(midi-keyboard 0)

# Add the standard library to the build
# pull in common dependencies

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(midi-keyboard PUBLIC pico_cyw43_arch_none)
endif()

# Make sure TinyUSB can find tusb_config.h
target_include_directories(midi-keyboard PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)
# Add any user requested libraries
target_link_libraries(midi-keyboard PUBLIC pico_stdlib pico_unique_id tinyusb_device tinyusb_board)

# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(midi-keyboard PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(midi-keyboard)

# add url via pico_set_program_url
# example_auto_set_url(midi-keyboard)
