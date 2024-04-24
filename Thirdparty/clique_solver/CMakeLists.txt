cmake_minimum_required(VERSION 3.5)
project(mac_solver)

include_directories(include)
FILE(GLOB_RECURSE ALL_SRC "src/*.c*" "include/*.h*")
add_library(${PROJECT_NAME} SHARED ${ALL_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_library(CLIQUE_SOLVER::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_subdirectory(Thirdparty/pmc)
target_link_libraries(${PROJECT_NAME} PMC::pmc_package)