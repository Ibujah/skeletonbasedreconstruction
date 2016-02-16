cmake_minimum_required(VERSION 2.6)

project(SkeletonBasedReconstruction)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Enable C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# Find the libraries
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(EIGEN REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VoroPlusPlus REQUIRED)

# Enable OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(MATHTOOLS_LIB  "Mathtools")
set(SHAPE_LIB      "Shape")
set(BOUNDARY_LIB   "Boundary")
set(CAMERA_LIB     "Camera")
set(ALGORITHM_LIB  "Algorithm")
set(SKELETON_LIB   "Skeleton")

set(USERINPUT_LIB  "UserInput")

set(DISPLAYOCV_LIB "DisplayOCV")

include_directories(${EIGEN_INCLUDE_DIRS})

add_subdirectory(src/)
