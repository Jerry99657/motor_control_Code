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
set(CMAKE_PROJECT_NAME STM32H743ZIT6_KIT)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB USER_DRIVER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/User/Src/*.c
)

file(GLOB_RECURSE LVGL_CORE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/lvgl/src/*.c
)

# Generated/third-party files retain upstream signatures and optional fallback
# code. Keep their known benign warnings out of the project warning baseline.
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c
    PROPERTIES COMPILE_OPTIONS "-Wno-missing-field-initializers"
)
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target/sd_diskio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target/bsp_driver_sd.c
    ${LVGL_CORE_SOURCES}
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter"
)
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/User/Src/mjpeg_player.c
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/lvgl/src/extra/libs/qrcode/qrcodegen.c
    PROPERTIES COMPILE_OPTIONS "-Wno-type-limits"
)
target_compile_options(FatFs PRIVATE -Wno-unused-parameter)

set(USER_PERSISTENT_SOURCES
    ${USER_DRIVER_SOURCES}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sd_service.c)
    list(APPEND USER_PERSISTENT_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sd_service.c
    )
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_PERSISTENT_SOURCES}
    ${LVGL_CORE_SOURCES}
)

# Add include paths
# Persistent user include directories (kept in top-level CMakeLists)
set(USER_PERSISTENT_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/User/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/lvgl
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_PERSISTENT_INCLUDE_DIRS}
)

# CubeMX regenerates Core/Inc/jpeg_utils_conf.h without USER CODE markers.
# Keep the project RGB565 configuration in Drivers/User/Inc and make it win
# for the separately compiled JPEG Utilities object library as well.
target_include_directories(Utilities BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/User/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    LV_CONF_INCLUDE_SIMPLE
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Treat fundamental C API mistakes as build failures while retaining normal
# warning visibility for generated middleware and third-party sources.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Werror=implicit-function-declaration
    -Werror=return-type
    -Werror=incompatible-pointer-types
)

# newlib-nano excludes floating-point printf/scanf conversion unless these
# symbols are explicitly retained. FOC UART4 command formatting uses %g and
# the USB/UART5 text command parser uses sscanf("%f").
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-u_printf_float"
    "-u_scanf_float"
)
