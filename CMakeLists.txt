cmake_minimum_required(VERSION 3.20)
project(jocat C)

set(CMAKE_C_STANDARD 99)

add_link_options(-pthread)

add_executable(jocat src/jocat.c)

