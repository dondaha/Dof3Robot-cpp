cmake_minimum_required(VERSION 3.16)

# 设置策略以消除 CMP0167 警告
cmake_policy(SET CMP0167 NEW)

# Project name
project(Dof3Robot LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
file(GLOB_RECURSE SOURCES src/*.cpp)

# Add include directories
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Create executable
add_executable(${PROJECT_NAME} src/main.cpp src/planner.cpp src/visualization.cpp)
add_executable(OMPL_TEST src/ompl_test.cpp)

# Add SFML as a dependency
find_package(SFML 3.0 COMPONENTS Graphics System REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE SFML::Graphics)

# Add OMPL as a dependency
find_package(ompl CONFIG REQUIRED)
## OMPL for OMPL_TEST
target_include_directories(OMPL_TEST PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(OMPL_TEST PRIVATE ompl)
## OMPL for main project
target_include_directories(${PROJECT_NAME} PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ompl)

# Enable OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

# Enable warnings
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
endif()
