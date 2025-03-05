cmake_minimum_required(VERSION 3.10.0)
project(vision VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV 4.0.0 REQUIRED)
find_package(X11 REQUIRED)

add_executable(vision 
    main.cpp 
    functions.h functions.cpp
)

target_link_libraries(vision ${OpenCV_LIBS})
target_link_libraries(vision ${X11_LIBRARIES})

include(CTest)
enable_testing()