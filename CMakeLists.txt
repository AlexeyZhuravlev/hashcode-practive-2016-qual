# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.5)

# Project's name

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(Google_Hash_Code_2020_Qualification)
# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/test)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


# flags
set (CMAKE_CXX_FLAGS "-fexceptions -std=c++17 -Wall -O3")

# The following folder will be included
include_directories(include)

add_subdirectory(src)
