cmake_minimum_required(VERSION 3.10)
project(pmc_package VERSION 1.0.0)

set(PMC_SOURCE_FILES
        src/pmc_heu.cpp
        src/pmc_maxclique.cpp
        src/pmcx_maxclique.cpp
        src/pmcx_maxclique_basic.cpp
        src/pmc_cores.cpp
        src/pmc_utils.cpp
        src/pmc_graph.cpp
        src/pmc_clique_utils.cpp
        )

add_library(${PROJECT_NAME} SHARED ${PMC_SOURCE_FILES})

find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )

add_library(PMC::${PROJECT_NAME} ALIAS ${PROJECT_NAME})