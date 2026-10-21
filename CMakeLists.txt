cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME cboard_gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    applications/led_task.cpp
    sp_middleware/io/led/led.cpp

    applications/buzzer_task.cpp
    sp_middleware/io/buzzer/buzzer.cpp

    applications/imu_task.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/tools/math_tools/math_tools.cpp

    applications/uart_task.cpp
    applications/tof_task.cpp
    sp_middleware/io/dbus/dbus.cpp

    applications/plotter_task.cpp
    sp_middleware/io/plotter/plotter.cpp

    applications/gimbal_task.cpp
    applications/fold_task.cpp
    applications/trajectory_task.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/tools/pid/pid.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    applications/
    sp_middleware/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
