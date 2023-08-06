cmake_minimum_required(VERSION 3.15)
project(voxl C)

find_package(GLFW3 REQUIRED)
find_package(PNG REQUIRED)

include_directories(include)

add_library(voxl OBJECT src/voxl.c)

add_library(polyconv OBJECT src/tribox.c src/polyconv.c)
target_link_libraries(polyconv PNG::PNG)

add_executable(objviewer samples/objviewer.c)
target_link_libraries(objviewer voxl polyconv glfw)

add_executable(benchmark samples/benchmark.c)
target_link_libraries(benchmark voxl polyconv glfw)
