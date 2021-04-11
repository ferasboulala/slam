cmake_minimum_required(VERSION 2.8.17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -Wextra -Wformat=0 -Werror=vla -DNDEBUG -O2 -finline-functions")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")

project(slam)

find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/slam")
file(GLOB PROJECT_SOURCES "${PROJECT_SOURCE_DIR}/slam/*.cpp")

enable_testing()
macro(unit_test test_name test_source)
    add_executable(${test_name} ${test_source} ${PROJECT_SOURCES})
    add_test(${test_name} ${CMAKE_CURRENT_SOURCE_DIR}/bin/${test_name})
    target_link_libraries(${test_name} -pthread ${GTEST_LIBRARIES})
endmacro()

unit_test("util_test" "test/test_util.cpp")
