cmake_minimum_required(VERSION 2.8.12)
project(hello)

add_definitions(-std=c++11)

# discrete filter, bearing only, obs2bin
add_executable(test1 filter/df.cpp vehicle/bearingonly.cpp mps_math.cpp test/test1.cpp)


# discrete filter, belief printing
add_executable(test2 filter/df.cpp vehicle/bearingonly.cpp mps_math.cpp test/test2.cpp)

# discrete filter, updating
add_executable(test3 filter/df.cpp vehicle/bearingonly.cpp vehicle/vehicle.cpp mps_math.cpp test/test3.cpp)

include_directories("${PROJECT_SOURCE_DIR}/bearing")
add_subdirectory("bearing")
add_executable(test4 circle.cpp filter/df.cpp vehicle/bearingonly.cpp vehicle/vehicle.cpp mps_math.cpp test/test4.cpp planner.cpp)

add_executable(test5 greedy.cpp filter/df.cpp vehicle/bearingonly.cpp vehicle/vehicle.cpp mps_math.cpp test/test5.cpp planner.cpp)

target_link_libraries(test4 BearingLib)
target_link_libraries(test5 BearingLib)
