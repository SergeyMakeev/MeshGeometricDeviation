cmake_minimum_required(VERSION 3.14)
project(MeshDeviance VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch fast_obj library
include(FetchContent)

FetchContent_Declare(
    fast_obj
    GIT_REPOSITORY https://github.com/thisistherk/fast_obj.git
    GIT_TAG master
)

FetchContent_MakeAvailable(fast_obj)

# Create mesh comparison tool executable
add_executable(mesh_compare 
    src/mesh_compare.cpp
    src/MeshComparison.cpp
    src/SpatialDb.cpp
)

# Link fast_obj library
target_link_libraries(mesh_compare PRIVATE fast_obj_lib)



