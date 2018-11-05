cmake_minimum_required(VERSION 3.0.0)

project(SQPController LANGUAGES CXX)

aux_source_directory(. src_file)

add_executable(SQPController ${src_file})
target_link_libraries(SQPController controllercommon)