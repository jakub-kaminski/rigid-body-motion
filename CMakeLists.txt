cmake_minimum_required(VERSION 3.16)

project(rigid-body-motion)

set(CMAKE_CXX_STANDARD 17)

include_directories(include/RBMotion)

add_subdirectory(lib)

add_subdirectory(src)

add_subdirectory(test)