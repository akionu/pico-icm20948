add_executable(ex2_madgwick ex2_madgwick.c)

target_link_libraries(ex2_madgwick pico_stdlib hardware_i2c pico-icm20948 MadgwickAHRS)

pico_enable_stdio_usb(ex2_madgwick 1)
pico_enable_stdio_uart(ex2_madgwick 0)

pico_add_extra_outputs(ex2_madgwick)