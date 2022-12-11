cmake_minimum_required(VERSION 3.10)
project(sim-device C)

set(CMAKE_C_STANDARD 99)

add_link_options(-pthread)

add_executable(sim-device ../src/sim-device.c ../src/serial-port.c)
