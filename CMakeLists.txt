cmake_minimum_required(VERSION 3.8)
project(FactorialCalculator)

include_directories(include)

add_executable(factorial 
    src/main.cpp 
    src/factorial.cpp
)
