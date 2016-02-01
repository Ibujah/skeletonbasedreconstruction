cmake_minimum_required(VERSION 2.6)

project(SkeletonBasedReconstruction)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Boost REQUIRED)
find_package(EIGEN REQUIRED)


add_subdirectory(src/)
