# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. This package is dependent on the simulator package, which can be retrieved from [...]

# Authors
Horia Andrei Leonte - Perception and image processing

Alexandre Justo Miro - Mission planning and path planning

alexandre.justomiro@studenti.unitn.it

# Dependencies - 1) CMake - 2) OpenCV - 3) Catkin - 4) etc.
sudo apt install cmake
sudo apt install libopencv-dev python3-opencv
sudo apt install catkin

# 5) GLPK - for mission 2
See: https://en.wikibooks.org/wiki/GLPK/Obtaining_GLPK
See: https://en.wikibooks.org/wiki/GLPK/Linux_OS
Find latest version here: http://ftp.gnu.org/gnu/glpk/
Note: in Ubuntu, after finishing the whole above process, the following command might be needed:
sudo ldconfig /usr/local/lib
Note: check the file CMakeLists.txt and verify that the GLPK library directory coincides with the actual location in your system.

# Every time any new files are added in the project library, report it to CMakeLists.txt.
All .h or .hpp files should go in the /inc folder, and all .cpp files should go in the /src directory.
Write all paths to .cpp files inside add_library() in CMakeLists.txt.

# Every time any files are added, or any changes are made to any existing files in the project library, recompile the library.
cd build
cmake ..
make

# Every time before starting a session, source environment in all the terminals you open.
cd project
source environment.sh

# Open simulator
AR_simulator # without graphical user interface
AR_simulator_gui # with graphical user interface

# Activate pipeline
AR_pipeline

# Compute planning
AR_plan

# Start motion
AR_run
