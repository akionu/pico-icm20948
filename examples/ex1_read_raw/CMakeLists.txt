add_executable(ex1_read_raw ex1_read_raw.c)

target_link_libraries(ex1_read_raw pico_stdlib hardware_i2c pico-icm20948)

pico_enable_stdio_usb(ex1_read_raw 1)
pico_enable_stdio_uart(ex1_read_raw 0)

pico_add_extra_outputs(ex1_read_raw)