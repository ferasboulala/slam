cmake_minimum_required(VERSION 2.8.17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -Wextra -Wformat=0 -Werror=vla -O3 -finline-functions")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")

project(slam)


# Packages
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories and src files
include_directories("${PROJECT_SOURCE_DIR}/slam")
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
file(GLOB PROJECT_SOURCES "${PROJECT_SOURCE_DIR}/slam/*.cpp" "${PROJECT_SOURCE_DIR}/slam/thirdparty/*.cpp")


# Tests
enable_testing()
macro(unit_test test_name test_source)
    add_executable(${test_name} ${test_source} ${PROJECT_SOURCES})
    add_test(${test_name} ${CMAKE_CURRENT_SOURCE_DIR}/bin/${test_name})
    target_link_libraries(${test_name} -pthread ${GTEST_LIBRARIES} -lopencv_core -lopencv_highgui -lopencv_imgproc -pthread)
endmacro()

unit_test("test_util" "test/test_util.cpp")
unit_test("test_raycast" "test/test_raycast.cpp")
unit_test("test_motion" "test/test_motion.cpp")

# Apps and libraries
macro(app progname source)
    add_executable(${progname} ${source} ${PROJECT_SOURCES})
    target_link_libraries(${progname} ${GTEST_LIBRARIES} ${OpenCV_LIBRARIES} -lopencv_core -lopencv_highgui -lopencv_imgproc -pthread)
endmacro()

app("motion_model" "apps/motion_model.cpp")
app("monte_carlo_localization" "apps/monte_carlo_localization.cpp")
