set(WORK_SPACE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

configure_file(
        ${PROJECT_SOURCE_DIR}/include/global_definition/global_definition.h.in
        ${PROJECT_BINARY_DIR}/include/global_definition/global_definition.h)
include_directories(${PROJECT_BINARY_DIR}/include)

message(PROJECT_BINARY_DIR " " ${PROJECT_BINARY_DIR})