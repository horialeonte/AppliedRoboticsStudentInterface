cmake_minimum_required(VERSION 2.8.3)
project(student_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 inc
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
 ## GLPK library
 /usr/local/include
)

## Add executable
add_executable(calibrate
   src/camera_calibration.cpp
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   src/robot_finder.cpp
   src/obstacle_detection.cpp
   src/victim_gate_detection.cpp
   src/dubins.cpp
   src/mission.cpp
   src/rrt_star.cpp
   src/clipper.cpp
   src/border_detection.cpp
   ## GLPK library
   /usr/local/lib/libglpk.a
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
  glpk
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}
)
