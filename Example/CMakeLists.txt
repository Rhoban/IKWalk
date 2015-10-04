###
### Humanoid IKWalk
### Rhoban 2015
###

cmake_minimum_required(VERSION 2.8)
project(IKWalk)

#Sources files
set(SOURCES_FILES_WALK
    ../CubicSpline.cpp
    ../HumanoidModel.cpp
    ../IKWalk.cpp
    ../LegIK.cpp
    ../Polynom.cpp
    ../Spline.cpp
)
set(SOURCES_FILES_EXAMPLE
    example.cpp
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

#Eigen library dependency
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#Enable compiler Warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")

#Build example application
add_executable(example 
    ${SOURCES_FILES_WALK} ${SOURCES_FILES_EXAMPLE})

