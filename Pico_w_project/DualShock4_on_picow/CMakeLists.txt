# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(DualShock4_on_picow C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialize the Raspberry Pi Pico SDK
add_definitions(-D PICO_FLASH_ASSERT_ON_UNSAFE=0)

pico_sdk_init()

add_subdirectory(lib)

# Add executable. Default name is the project name

add_executable(DualShock4_on_picow
  main.cpp
)

pico_set_program_name(DualShock4_on_picow "DualShock4_on_picow")
pico_set_program_version(DualShock4_on_picow "0.2.0")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(DualShock4_on_picow 0)
pico_enable_stdio_usb(DualShock4_on_picow 1)

# Add the standard library to the build
target_link_libraries(DualShock4_on_picow
    pico_stdlib
)

# Add the standard include files to the build
target_include_directories(DualShock4_on_picow PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(DualShock4_on_picow
    lib_ds4_on_pico
)

pico_add_extra_outputs(DualShock4_on_picow)
