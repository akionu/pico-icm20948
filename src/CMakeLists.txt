add_library(pico-icm20948 INTERFACE)

target_sources(pico-icm20948 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico-icm20948.c
)

target_link_libraries(pico-icm20948
    INTERFACE
    pico_stdlib
    hardware_i2c
)