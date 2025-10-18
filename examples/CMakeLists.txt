# Examples for MeshGeometricDeviation library

add_executable(mesh_compare mesh_compare.cpp)
target_link_libraries(mesh_compare PRIVATE MeshGeometricDeviation)

# On Windows, copy DLLs to the executable directory
if(WIN32)
    add_custom_command(TARGET mesh_compare POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:MeshGeometricDeviation>
        $<TARGET_FILE_DIR:mesh_compare>
    )
endif()

