cmake_minimum_required(VERSION 3.12)
project(Common)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core CONFIG)

aux_source_directory(. Common_SRCS)

add_library(Common SHARED ${Common_SRCS})
target_link_libraries(Common Qt5::Widgets Qt5::Core)