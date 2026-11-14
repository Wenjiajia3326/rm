cmake_minimum_required(VERSION 3.22)

# 工具链文件必须在本文件最前面加载（先于 enable_language/project），
# 这样 CMake 才知道目标是 arm-none-eabi 裸机工程，而不是 Windows 程序。
# 这样 CLion / VS Code / 命令行都不需要额外配置 CMake profile。
include("${CMAKE_CURRENT_LIST_DIR}/cmake/gcc-arm-none-eabi.cmake")

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
set(CMAKE_PROJECT_NAME rm)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
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
# 用户代码分层：application / components / bsp
# 目录为空时 glob 结果为空，不影响构建；新增 .c 后重新配置即可自动纳入
file(GLOB_RECURSE USER_SRCS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/application/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SRCS}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/application
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
