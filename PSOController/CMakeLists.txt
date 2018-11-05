cmake_minimum_required(VERSION 3.0.0)

project(PSOController LANGUAGES CXX)

aux_source_directory(. src_file)

add_executable(PSOController ${src_file})
target_link_libraries(PSOController controllercommon)