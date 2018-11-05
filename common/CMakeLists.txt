cmake_minimum_required(VERSION 3.0.0)

project(common LANGUAGES CXX)

file(GLOB source_file
	"*.h"
	"*.hpp"
	"*.cpp"
)

add_library(controllercommon STATIC ${source_file})
