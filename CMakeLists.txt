# get all the files that make up bearing lib
file(GLOB SOURCES "*.cpp" "filter/*.cpp" "vehicle/*.cpp")

include_directories(filter)
include_directories(vehicle)
add_library(PlannerLib ${SOURCES})
