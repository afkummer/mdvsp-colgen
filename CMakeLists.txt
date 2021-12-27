cmake_minimum_required(VERSION 3.0)

project(mdvsp-colgen CXX)

add_definitions(
   -Wall 
   -Wextra 
   -std=c++17
   -ggdb3
)

include_directories(src)

add_executable(main
   src/main.cpp
   src/Instance.cpp
)
