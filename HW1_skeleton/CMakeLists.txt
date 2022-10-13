cmake_minimum_required (VERSION 2.8.11)
project (assignment_1)

# set EIGEN_DIR variable to Eigen Library Path
set(EIGEN_DIR_ALT ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen-3.4.0)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN_DIR_ALT})


add_executable (assignment1 main.cpp)
