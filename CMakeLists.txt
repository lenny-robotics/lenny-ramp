# Minimum requirements for cmake
cmake_minimum_required(VERSION 3.22)

# CMake helpers
add_subdirectory(cmake)

# Project name
project(lenny-ramp)

# Build option
capitalize(LENNY_PROJECT ${PROJECT_NAME})
option(${LENNY_PROJECT}_BUILD_APPS "Build apps" OFF)

# Project folder path
set(LENNY_PROJECT_FOLDER "\"${CMAKE_CURRENT_LIST_DIR}\"" CACHE STRING "")

# Generate log directory
file(MAKE_DIRECTORY "logs")

# CXX standard
set(CMAKE_CXX_STANDARD 20)

# Fetch lenny repository
fetch_lenny_repository(robot master)

# Source code
add_subdirectory(source)
