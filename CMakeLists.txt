#==============================================================================
# SS22 Project Test Task
#==============================================================================

#------------------------------------------------------------------------------
# CMake requirements
#------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.13)

#------------------------------------------------------------------------------
# Compiler requirements
#------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)

#------------------------------------------------------------------------------
# Project metadata
#------------------------------------------------------------------------------
project(SS22_PROJECT_TEST)

#------------------------------------------------------------------------------
# Path settings
#------------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/obj")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(DEFAULT_TARGET_INSTALL_DESTINATIONS
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )

#------------------------------------------------------------------------------
# Required packages
#------------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)

#------------------------------------------------------------------------------
# Process sources
#------------------------------------------------------------------------------
include_directories(BEFORE
        "${PROJECT_SOURCE_DIR}/src"
        "${PROJECT_BINARY_DIR}/src"
        )

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)