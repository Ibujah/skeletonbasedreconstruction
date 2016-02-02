cmake_minimum_required(VERSION 2.6)

project(SkeletonBasedReconstruction)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Enable C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Boost REQUIRED)
find_package(EIGEN REQUIRED)

set(MATHTOOLS_LIB "Mathtools_Lib")

include_directories(${Boost_INCLUDE_DIR}
					${EIGEN_INCLUDE_DIRS})

add_subdirectory(src/)
