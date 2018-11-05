cmake_minimum_required(VERSION 3.0.0)

project(controller LANGUAGES CXX)

add_subdirectory(common)

include_directories(${common_SOURCE_DIR})

add_subdirectory(PSOController)
add_subdirectory(SQPController)