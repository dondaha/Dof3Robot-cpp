cmake_minimum_required(VERSION 3.16)

# Project name
project(Dof3Robot LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES src/*.cpp)

# Add include directories
include_directories(include third_party/eigen-3.4.0)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add SFML as a dependency
include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.1
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)
target_link_libraries(${PROJECT_NAME} PRIVATE SFML::Graphics)

# Enable OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

# Enable warnings
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
endif()