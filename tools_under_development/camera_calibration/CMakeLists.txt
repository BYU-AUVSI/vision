cmake_minimum_required(VERSION 2.8.12)
project(CalibrateCamera)
FIND_PACKAGE(OpenCV REQUIRED)


add_compile_options(-std=c++0x)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)


SET(MYSOURCE
    calibrate_camera.cpp
    )

add_executable(CalibrateCamera WIN32
    ${MYSOURCE}
    )

target_link_libraries(CalibrateCamera
    ${OpenCV_LIBS}
    )
