cmake_minimum_required(VERSION 3.5)
project(fpfh_matcher)

include(GNUInstallDirs)

FILE(GLOB_RECURSE ALL_SRC "src/*.c*" "include/*.h*")
add_library(${PROJECT_NAME} SHARED ${ALL_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_library(FPFH_MATCHER::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT FPFH_MATCHERConfig
  LIBRARY DESTINATION lib
)

export(TARGETS ${PROJECT_NAME}
  NAMESPACE FPFH_MATCHER::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/FPFH_MATCHERConfig.cmake"
)
install(EXPORT FPFH_MATCHERConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/FPFH_MATCHER/cmake"
  NAMESPACE FPFH_MATCHER::
)