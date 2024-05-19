cmake_minimum_required(VERSION 3.15)
project(g3reg)

message("start make g3reg")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/yaml.cmake)
include(cmake/glog.cmake)
include(cmake/pcl.cmake)
include(cmake/openmp.cmake)
include(cmake/gtsam.cmake)
include(cmake/global_definition.cmake)
include(cmake/igraph.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)
file(GLOB_RECURSE ALL_SRCS "src/*.c*" "include/*.h*")
list(APPEND ALL_TARGET_LIBRARIES stdc++fs tbb)
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_subdirectory(Thirdparty/clique_solver)
add_subdirectory(Thirdparty/robot_utils)
add_subdirectory(Thirdparty/fpfh_matcher)
add_subdirectory(Thirdparty/backward-cpp)
target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES}
        CLIQUE_SOLVER::mac_solver
        ROBOT_UTILS::robot_utils
        FPFH_MATCHER::fpfh_matcher
)
message(STATUS "ALL_TARGET_LIBRARIES: ${ALL_TARGET_LIBRARIES}")
add_library(G3REG::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# set bin output path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(reg_bm examples/reg_bm.cpp ${BACKWARD_ENABLE})
add_backward(reg_bm)
target_link_libraries(reg_bm ${PROJECT_NAME})

add_executable(matching_bm examples/matching_bm.cpp ${BACKWARD_ENABLE})
add_backward(matching_bm)
target_link_libraries(matching_bm ${PROJECT_NAME})

add_executable(demo_reg examples/demo_reg.cpp ${BACKWARD_ENABLE})
add_backward(demo_reg)
target_link_libraries(demo_reg ${PROJECT_NAME})

add_executable(demo_seg examples/demo_seg.cpp ${BACKWARD_ENABLE})
add_backward(demo_seg)
target_link_libraries(demo_seg ${PROJECT_NAME})