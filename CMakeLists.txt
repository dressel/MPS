# get all the files that make up bearing lib
file(GLOB SOURCES "*.cpp" "filter/*.cpp" "vehicle/*.cpp")

add_library(PlannerLib ${SOURCES})
