# get all the files that make up bearing lib
file(GLOB SOURCES "*.cpp")

add_library(BearingLib ${SOURCES})