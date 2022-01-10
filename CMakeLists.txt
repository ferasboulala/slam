cmake_minimum_required(VERSION 2.8.17)

project(slam)

add_compile_options(-Ofast -fno-math-errno -finline-functions -Wall -Wextra -Wpedantic -std=c++17 -std=gnu++17) #-Werror)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")


find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)


add_library(slam STATIC
	slam/astar.cpp
	slam/astar.h
	slam/colors.h
	slam/fake_lidar.cpp
	slam/fake_lidar.h
	slam/hastar.cpp
	slam/hastar.h
	slam/kdtree.cpp
	slam/kdtree.h
	slam/mcl.cpp
	slam/mcl.h
	slam/motion.cpp
	slam/motion.h
	slam/pose.h
	slam/quadtree.cpp
	slam/quadtree.h
	slam/raycast.cpp
	slam/raycast.h
	slam/rrtstar.cpp
	slam/rrtstar.h
	slam/thirdparty
	slam/util.cpp
	slam/util.h
	slam/thirdparty/log.h
	slam/thirdparty/log.cpp
)
target_include_directories(slam PRIVATE slam ${OpenCV_INCLUDE_DIRS})
target_link_libraries(slam ${OpenCV_LIBRARIES} -pthread)


macro(app progname source)
    add_executable(${progname} ${source} ${PROJECT_SOURCES})
	target_include_directories(${progname} PRIVATE ${OpenCV_INCLUDE_DIRS} slam)
    target_link_libraries(${progname} ${OpenCV_LIBRARIES} -lopencv_core -lopencv_highgui -lopencv_imgproc -pthread slam)
endmacro()


app("grid_slam" "apps/monte_carlo_localization.cpp")
app("rrt_planner" "apps/rrt_planner.cpp")
app("hastar_planner" "apps/hastar_planner.cpp")
app("nearest_neighbor" "apps/nearest_neighbor.cpp")
app("regions" "apps/regions.cpp")
