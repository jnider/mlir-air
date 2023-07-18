cmake_minimum_required(VERSION 3.20)

project(amdair_test)

set(CMAKE_CXX_STANDARD 20)
add_executable(air_test main.cpp)
find_package(hsa-runtime64 REQUIRED)
target_link_libraries(air_test PRIVATE hsa-runtime64::hsa-runtime64)
