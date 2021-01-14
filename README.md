# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course.

# Authors
Horia Andrei Leonte
Alexandre Justo Miro

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

# Every time any new files are added in the project library, report it to CMakeLists.txt.
All .hpp files should go in the /include folder, and .cpp files should go in the /src directory.
Write all paths to cpp files inside add_library() in CMakeLists.txt.

# Every time any files are added, or any changes are made to any existing file in the project library, recompile the library.
cd project/build
cmake ..
make

# Every time before starting a session, source environment
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
