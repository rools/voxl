cmake_minimum_required(VERSION 3.15)
project(voxl CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(GLFW3 REQUIRED)
find_package(PNG REQUIRED)
find_package(HIGHWAY REQUIRED)

include_directories(include)

add_library(voxl OBJECT src/voxl.cpp)
target_link_libraries(voxl highway::hwy)

add_library(polyconv OBJECT src/tribox.cpp src/polyconv.cpp)
target_link_libraries(polyconv PNG::PNG)

add_executable(objviewer samples/objviewer.cpp)
target_link_libraries(objviewer voxl polyconv glfw)

add_executable(benchmark samples/benchmark.cpp)
target_link_libraries(benchmark voxl polyconv glfw)
