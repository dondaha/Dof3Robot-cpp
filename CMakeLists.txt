cmake_minimum_required(VERSION 3.16)

# Project name
project(Dof3Robot LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES src/*.cpp)

# Add include directories
include_directories(include)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(main PRIVATE SFML::Graphics)

# Enable warnings
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
endif()