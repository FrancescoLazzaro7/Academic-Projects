cmake_minimum_required(VERSION 3.20)
project(odeint)

set(CMAKE_CXX_STANDARD 20)

add_executable(simulator main.cpp simulator_odefun.cpp)
