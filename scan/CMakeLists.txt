cmake_minimum_required(VERSION 3.0)
project(scan)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV REQUIRED)
include_directories(. ${OpenCV_INCLUDE_DIRS})

add_executable(
    scan
    main.cpp
    match.h
    match.cpp
    scan.h
    scan.cpp
)
target_link_libraries(scan ${OpenCV_LIBS})
