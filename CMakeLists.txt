# CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(Sysguage VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src)

# Add executable
add_executable(sysguage src/sysguage.cpp)

# Specify platform-specific settings if needed
if(WIN32)
    # Windows-specific configurations
    target_compile_definitions(sysguage PRIVATE PLATFORM_WINDOWS)
elseif(APPLE)
    # macOS-specific configurations
    target_compile_definitions(sysguage PRIVATE PLATFORM_MACOS)
elseif(UNIX)
    # Linux-specific configurations
    target_compile_definitions(sysguage PRIVATE PLATFORM_LINUX)
endif()
