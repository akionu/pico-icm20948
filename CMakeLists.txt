#Set the mimum required version of CMake
cmake_minimum_required(VERSION 3.12)

#Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

#Set the project name, the programming language
project(project1 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#Initialize the SDK
pico_sdk_init()

#add_subdirectory("Fusion")
add_subdirectory("MadgwickAHRS")

#Add an executable target to be build from the source files
add_executable(test test.c)

#Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(test pico_stdlib hardware_i2c MadgwickAHRS)

#Enable USB output, disable UART output
pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 0)

#Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(test)