cmake_minimum_required(VERSION 3.1)
project(follow)

set(CMAKE_GXX_FLAGS "-Wall -Wextra -Wconversion  -pedantic -std=gnu11")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(yeet ./follow.cpp)
target_link_libraries(yeet ${OpenCV_LIBS})