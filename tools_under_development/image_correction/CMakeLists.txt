cmake_minimum_required(VERSION 2.8.12)
project(ImageCorrection)
FIND_PACKAGE(OpenCV REQUIRED)


add_compile_options(-std=c++0x)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)


SET(MYSOURCE
    image_correction.cpp
    )

add_executable(ImageCorrection WIN32
    ${MYSOURCE}
    )

target_link_libraries(ImageCorrection
    ${OpenCV_LIBS}
    )
