cmake_minimum_required(VERSION 3.10.0)
project(ForSimu VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV 4.0.0 REQUIRED)

add_executable(ForSimu main.cpp
    functions.h functions.cpp
    imagehandler.h imagehandler.cpp
    linedetecter.h linedetecter.cpp
)

target_link_libraries(ForSimu ${OpenCV_LIBS})

include(CTest)
enable_testing()
