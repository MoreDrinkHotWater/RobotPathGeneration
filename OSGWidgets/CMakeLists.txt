cmake_minimum_required(VERSION 3.12)
project(OSGWidgets)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Qt related
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL CONFIG)

find_package(OpenSceneGraph 3.2 REQUIRED COMPONENTS osgGA osgViewer osgDB osgUtil osg osgQt osgFX)
find_package(PCL REQUIRED)
# C++和C版本的都要安装，不然会报错找不到capi/version.h文件，其中库文件地址简写为libLAS_LIBRARIES，而不是LIBLAS_LIBRARIES
find_package(libLAS REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

aux_source_directory(. OSGWidgets_SRCS)

add_library(OSGWidgets SHARED ${OSGWidgets_SRCS} JudgeGroundPoint.h JudgeGroundPoint.cpp Exception.h)
target_include_directories(OSGWidgets PUBLIC ../Common ${OPENSCENEGRAPH_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${LIBLAS_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(OSGWidgets Qt5::Widgets Qt5::Core Common ${OPENSCENEGRAPH_LIBRARIES} ${PCL_LIBRARIES} ${libLAS_LIBRARIES} ${OpenCV_LIBS})
