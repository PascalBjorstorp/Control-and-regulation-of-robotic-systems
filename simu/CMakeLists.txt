cmake_minimum_required(VERSION 3.10)
project(BallMazeGame)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# Executable
add_executable(BallMazeGame main.cpp
    ball.h ball.cpp
    wall.h wall.cpp
    point3d.h point3d.cpp
    maze.h maze.cpp
    motor.h motor.cpp
    updater.h updater.cpp
    imagehandler.h imagehandler.cpp
    linedetecter.h linedetecter.cpp
    Constants.h
    uartcom.h uartcom.cpp
)

# Link SFML libraries
target_link_libraries(BallMazeGame PRIVATE sfml-graphics sfml-window sfml-system ${OpenCV_LIBS} opencv_core opencv_imgproc)
