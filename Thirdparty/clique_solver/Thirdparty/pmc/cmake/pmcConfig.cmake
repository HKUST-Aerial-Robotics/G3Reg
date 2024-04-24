get_filename_component(PMC_CMAKE_DIR "pmcConfig.cmake" PATH)

include(CMakeFindDependencyMacro)

find_dependency(OpenMP REQUIRED)

include("${PMC_CMAKE_DIR}/pmcTargets.cmake")
