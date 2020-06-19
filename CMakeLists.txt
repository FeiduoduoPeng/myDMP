cmake_minimum_required(VERSION 2.8.3)
project(dmp)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(
  "/usr/include/eigen3"
  include/${PROJECT_NAME}
  include
)


add_library(dmp src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp src/linear_approx.cpp)
add_executable(dmp_server nodes/dmp_server.cpp)
target_link_libraries(dmp_server dmp)
