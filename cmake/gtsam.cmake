find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
list(APPEND ALL_TARGET_LIBRARIES gtsam)