cmake_minimum_required(VERSION 3.5)
project(robot_utils)

include(GNUInstallDirs)

FILE(GLOB_RECURSE ALL_SRC "src/*.c*" "include/*.h*")
add_library(${PROJECT_NAME} SHARED ${ALL_SRC} include/robot_utils/algorithms.h)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

find_package(PCL REQUIRED COMPONENTS common io)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
add_library(ROBOT_UTILS::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT ROBOT_UTILSConfig
  LIBRARY DESTINATION lib
)

export(TARGETS ${PROJECT_NAME}
  NAMESPACE ROBOT_UTILS::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/ROBOT_UTILSConfig.cmake"
)
install(EXPORT ROBOT_UTILSConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/ROBOT_UTILS/cmake"
  NAMESPACE ROBOT_UTILS::
)