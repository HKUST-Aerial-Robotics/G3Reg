#include_directories("third_party/eigen3")

find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

include_directories(
        ${Boost_INCLUDE_DIRS}
)
list(APPEND ALL_TARGET_LIBRARIES ${Boost_LIBRARIES})
