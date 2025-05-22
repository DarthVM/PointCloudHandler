cmake_minimum_required(VERSION 3.27)
project(PointCloudHandler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH Libraries/x64-windows)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")
set(CMAKE_VERBOSE_MAKEFILE ON)


find_package(CGAL)

find_package(Eigen3 3.1.0 QUIET) #(requires 3.1.0 or greater)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
    message(FATAL_ERROR "NOTICE: This project requires Eigen 3.1 (or greater), and will not be compiled.")
    return()
endif()

find_package(GMP REQUIRED)

add_executable(${PROJECT_NAME}
        Sources/main.cpp
        Sources/Primitives/Cloud/PointCloud.cpp
        Sources/Primitives/Cloud/CloudReconstruction.cpp
        Sources/Primitives/Model/Model.cpp
        Sources/Primitives/Model/ModelShapeDetection.cpp
        Sources/Primitives/MeshPlaneDetector/Metaplane.cpp
        Sources/Primitives/MeshPlaneDetector/MeshPlaneDetector.cpp
        Sources/Primitives/MeshPlaneDetector/Utils.cpp
        Headers/Primitives/Math.hpp
)


target_link_libraries(${PROJECT_NAME}
        PRIVATE CGAL::CGAL
        PUBLIC CGAL::Eigen3_support
        ${GMP_LIBRARIES}
)

include_directories(${CMAKE_PREFIX_PATH}/include)


#[[
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${CMAKE_PREFIX_PATH}/bin/*.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path]]
