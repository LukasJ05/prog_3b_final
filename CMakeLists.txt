cmake_minimum_required(VERSION 3.31)
project(prog_3b_final)

set(CMAKE_CXX_STANDARD 20)

add_executable(prog_3b_final src/main.cpp
        src/Graph.cpp
        src/Graph.h
        src/Edge.cpp
        src/Edge.h
        src/Pathfinder.cpp
        src/Pathfinder.h)

file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})