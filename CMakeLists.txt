cmake_minimum_required(VERSION 3.14)
project(MeshGeometricDeviation VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Option to build examples
option(BUILD_EXAMPLES "Build example applications" ON)

# Fetch fast_obj library
include(FetchContent)

FetchContent_Declare(
    fast_obj
    GIT_REPOSITORY https://github.com/thisistherk/fast_obj.git
    GIT_TAG master
)

FetchContent_MakeAvailable(fast_obj)

# Library target
add_library(MeshGeometricDeviation
    src/MeshComparison.cpp
    src/SpatialDb.cpp
)

target_include_directories(MeshGeometricDeviation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(MeshGeometricDeviation
    PRIVATE
        fast_obj_lib
)

# Copy fast_obj header to our include directory for distribution
target_include_directories(MeshGeometricDeviation
    PRIVATE
        ${fast_obj_SOURCE_DIR}
)

# Set library properties
set_target_properties(MeshGeometricDeviation PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "include/MeshGeometricDeviation/MeshComparison.h;include/MeshGeometricDeviation/SpatialDb.h"
)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation rules
include(GNUInstallDirs)

install(TARGETS MeshGeometricDeviation
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/MeshGeometricDeviation
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
