cmake_minimum_required(VERSION 3.12)
project(PointCloudApplication)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Xml CONFIG)

find_package(PCL REQUIRED)

aux_source_directory(. PointCloudApplication_SRCS)

list(APPEND PointCloudApplication_SRCS
        Toolbar.h
        Toolbar.cpp)

set(RESOURCE_DIR ../ui/icon.qrc)

add_executable(PointCloudApplication ${PointCloudApplication_SRCS} ${RESOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_include_directories(PointCloudApplication PRIVATE ../Common ../OSGWidgets)
target_link_libraries(PointCloudApplication Qt5::Widgets Qt5::Xml Common OSGWidgets ${PCL_LIBRARIES})
