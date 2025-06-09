cmake_minimum_required(VERSION 3.16)
project(libcarla_c VERSION 0.10.0 LANGUAGES C CXX)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set C++ standard for our C wrapper (C++20 required for CARLA 0.10.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find the installed CARLA library
# Users should either:
# 1. Set CMAKE_PREFIX_PATH to the CARLA installation directory
# 2. Set CARLA_DIR to the directory containing CarlaConfig.cmake
# 3. Install CARLA to a standard system location
find_package(Carla REQUIRED CONFIG)

message(STATUS "Found CARLA at: ${Carla_DIR}")

# Source files for C wrapper
set(SOURCES
    src/client.cpp
    src/world.cpp
    src/map.cpp
    src/actor.cpp
    src/vehicle.cpp
    src/sensor.cpp
    src/motion.cpp
    src/lidar.cpp
    src/camera.cpp
    src/detection.cpp
    src/traffic_manager.cpp
    src/common.cpp
)

# Create shared library for C wrapper
add_library(carla_c SHARED ${SOURCES})

# Include directories
target_include_directories(carla_c 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link against CARLA client library
target_link_libraries(carla_c
    PUBLIC
        Carla::carla-client-interface
)

# Set library properties
set_target_properties(carla_c PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    PUBLIC_HEADER "include/carla_c/client.h;include/carla_c/types.h;include/carla_c/world.h;include/carla_c/map.h;include/carla_c/actor.h;include/carla_c/vehicle.h;include/carla_c/sensor.h;include/carla_c/motion.h;include/carla_c/lidar.h;include/carla_c/camera.h;include/carla_c/detection.h;include/carla_c/traffic_manager.h"
)

# Installation rules
install(TARGETS carla_c
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include/carla_c
)

# Export compile commands for development
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable parallel compilation
include(ProcessorCount)
ProcessorCount(N)
if(NOT N EQUAL 0)
    set(CMAKE_BUILD_PARALLEL_LEVEL ${N})
    message(STATUS "Parallel build level set to ${N}")
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add example executable
add_executable(carla_c_example example.c)
target_link_libraries(carla_c_example PRIVATE carla_c)

# Print configuration summary
message(STATUS "")
message(STATUS "libcarla_c Configuration Summary")
message(STATUS "================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Parallel jobs: ${CMAKE_BUILD_PARALLEL_LEVEL}")
message(STATUS "CARLA found at: ${Carla_DIR}")
message(STATUS "")
message(STATUS "Build outputs:")
message(STATUS "  Libraries: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "  Binaries: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "")

# Add custom target for easy building
add_custom_target(build-all ALL
    COMMENT "Building libcarla_c and all dependencies..."
)
add_dependencies(build-all carla_c carla_c_example)

# Add convenience targets
add_custom_target(run-example
    COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/carla_c_example
    DEPENDS carla_c_example
    COMMENT "Running carla_c example..."
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Print build instructions
message(STATUS "To build: cmake --build . --config Release")
message(STATUS "To run example: cmake --build . --target run-example")
message(STATUS "")
message(STATUS "Build requirements:")
message(STATUS "  1. CARLA C++ library must be installed first")
message(STATUS "  2. Set CMAKE_PREFIX_PATH to CARLA install directory, or")
message(STATUS "     Set CARLA_DIR to directory containing CarlaConfig.cmake")
message(STATUS "")
message(STATUS "Example build commands:")
message(STATUS "  cmake -B build -S . -DCMAKE_PREFIX_PATH=/path/to/carla/install")
message(STATUS "  cmake --build build")
message(STATUS "")

# Post-build summary function
function(print_build_summary)
    message(STATUS "")
    message(STATUS "🎉 libcarla_c build configuration complete!")
    message(STATUS "")
    message(STATUS "Quick start commands:")
    message(STATUS "  Build all:     cmake --build ${CMAKE_BINARY_DIR}")
    message(STATUS "  Run example:   cmake --build ${CMAKE_BINARY_DIR} --target run-example")
    message(STATUS "  Install:       cmake --build ${CMAKE_BINARY_DIR} --target install")
    message(STATUS "")
endfunction()

# Call summary at the end
cmake_language(DEFER CALL print_build_summary)