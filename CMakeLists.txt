cmake_minimum_required(VERSION 3.7)
project(visual-slam LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif ()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Eigen
include_directories("/usr/local/include/eigen3")
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
SET(G2O_LIBS
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_types_sim3
        g2o_solver_dense
        g2o_solver_eigen
        g2o_solver_csparse
        g2o_csparse_extension)
# CSparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
# FBoW
find_package(fbow REQUIRED)
include_directories(${fbow_INCLUDE_DIRS})
# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/popl)
# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIR})
# library
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        ${CSPARSE_LIBRARY}
        yaml-cpp
        ${fbow_LIBS}
        ${Pangolin_LIBRARIES})
# include
include_directories(${PROJECT_SOURCE_DIR}/include)
# source
set(SRC_CORE
        ${PROJECT_SOURCE_DIR}/src/config.cpp
        ${PROJECT_SOURCE_DIR}/src/global_optimization_module.cpp
        ${PROJECT_SOURCE_DIR}/src/mapping_module.cpp
        ${PROJECT_SOURCE_DIR}/src/system.cpp
        ${PROJECT_SOURCE_DIR}/src/tracking_module.cpp)
set(SRC_CAMERA
        ${PROJECT_SOURCE_DIR}/src/camera/base.cpp
        ${PROJECT_SOURCE_DIR}/src/camera/perspective.cpp)
set(SRC_DATA
        ${PROJECT_SOURCE_DIR}/src/data/bow_database.cpp
        ${PROJECT_SOURCE_DIR}/src/data/camera_database.cpp
        ${PROJECT_SOURCE_DIR}/src/data/common.cpp
        ${PROJECT_SOURCE_DIR}/src/data/frame.cpp
        ${PROJECT_SOURCE_DIR}/src/data/frame_statistics.cpp
        ${PROJECT_SOURCE_DIR}/src/data/graph_node.cpp
        ${PROJECT_SOURCE_DIR}/src/data/keyframe.cpp
        ${PROJECT_SOURCE_DIR}/src/data/landmark.cpp
        ${PROJECT_SOURCE_DIR}/src/data/map_database.cpp)
set(SRC_FEATURE
        ${PROJECT_SOURCE_DIR}/src/feature/orb_extractor.cpp
        ${PROJECT_SOURCE_DIR}/src/feature/orb_extractor_node.cpp
        ${PROJECT_SOURCE_DIR}/src/feature/orb_params.cpp)
set(SRC_INITIALIZE
        ${PROJECT_SOURCE_DIR}/src/initialize/base.cpp
        ${PROJECT_SOURCE_DIR}/src/initialize/bearing_vector.cpp
        ${PROJECT_SOURCE_DIR}/src/initialize/perspective.cpp)
set(SRC_IO
        ${PROJECT_SOURCE_DIR}/src/io/map_database_io.cpp
        ${PROJECT_SOURCE_DIR}/src/io/trajectory_io.cpp)
set(SRC_MATCH
        ${PROJECT_SOURCE_DIR}/src/match/area.cpp
        ${PROJECT_SOURCE_DIR}/src/match/bow_tree.cpp
        ${PROJECT_SOURCE_DIR}/src/match/fuse.cpp
        ${PROJECT_SOURCE_DIR}/src/match/projection.cpp
        ${PROJECT_SOURCE_DIR}/src/match/robust.cpp
        ${PROJECT_SOURCE_DIR}/src/match/stereo.cpp)
set(SRC_MODULE
        ${PROJECT_SOURCE_DIR}/src/module/frame_tracker.cpp
        ${PROJECT_SOURCE_DIR}/src/module/initializer.cpp
        ${PROJECT_SOURCE_DIR}/src/module/keyframe_inserter.cpp
        ${PROJECT_SOURCE_DIR}/src/module/local_map_cleaner.cpp
        ${PROJECT_SOURCE_DIR}/src/module/local_map_updater.cpp
        ${PROJECT_SOURCE_DIR}/src/module/loop_bundle_adjuster.cpp
        ${PROJECT_SOURCE_DIR}/src/module/loop_detector.cpp
        ${PROJECT_SOURCE_DIR}/src/module/relocalizer.cpp
        ${PROJECT_SOURCE_DIR}/src/module/two_view_triangulator.cpp)
set(SRC_OPTIMIZE
        ${PROJECT_SOURCE_DIR}/src/optimize/global_bundle_adjuster.cpp
        ${PROJECT_SOURCE_DIR}/src/optimize/graph_optimizer.cpp
        ${PROJECT_SOURCE_DIR}/src/optimize/local_bundle_adjuster.cpp
        ${PROJECT_SOURCE_DIR}/src/optimize/pose_optimizer.cpp
        ${PROJECT_SOURCE_DIR}/src/optimize/transform_optimizer.cpp)
set(SRC_PUBLISH
        ${PROJECT_SOURCE_DIR}/src/publish/frame_publisher.cpp
        ${PROJECT_SOURCE_DIR}/src/publish/map_publisher.cpp)
set(SRC_SOLVE
        ${PROJECT_SOURCE_DIR}/src/solve/common.cpp
        ${PROJECT_SOURCE_DIR}/src/solve/essential_solver.cpp
        ${PROJECT_SOURCE_DIR}/src/solve/fundamental_solver.cpp
        ${PROJECT_SOURCE_DIR}/src/solve/homography_solver.cpp
        ${PROJECT_SOURCE_DIR}/src/solve/pnp_solver.cpp
        ${PROJECT_SOURCE_DIR}/src/solve/sim3_solver.cpp)
set(SRC_UTIL
        ${PROJECT_SOURCE_DIR}/src/util/converter.cpp
        ${PROJECT_SOURCE_DIR}/src/util/image_converter.cpp
        ${PROJECT_SOURCE_DIR}/src/util/image_sequence.cpp
        ${PROJECT_SOURCE_DIR}/src/util/random_array.cpp
        ${PROJECT_SOURCE_DIR}/src/util/stereo_rectifier.cpp)
set(SRC_VIEWER
        ${PROJECT_SOURCE_DIR}/include/viewer/viewer.cpp
        ${PROJECT_SOURCE_DIR}/include/viewer/color_scheme.cpp)

set(LOCAL_SRC
        ${SRC_CORE}
        ${SRC_CAMERA}
        ${SRC_DATA}
        ${SRC_FEATURE}
        ${SRC_INITIALIZE}
        ${SRC_IO}
        ${SRC_MATCH}
        ${SRC_MODULE}
        ${SRC_OPTIMIZE}
        ${SRC_PUBLISH}
        ${SRC_SOLVE}
        ${SRC_UTIL}
        ${SRC_VIEWER})

add_executable(visual-slam main.cpp ${LOCAL_SRC})

target_link_libraries(visual-slam ${THIRD_PARTY_LIBS})
