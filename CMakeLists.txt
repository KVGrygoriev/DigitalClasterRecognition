cmake_minimum_required(VERSION 3.5)

project(DigitalClasterRecognition LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( Threads )

# POC files
message("copy to ${CMAKE_BINARY_DIR}")
file(COPY poc DESTINATION  ${CMAKE_BINARY_DIR})

# OpenCV
# https://opencv.org
find_package(OpenCV 3.3.1 REQUIRED
  COMPONENTS
    core
    imgcodecs
    imgproc
    videoio
    highgui
)

add_executable(DigitalClasterRecognition
  AnalogMeterDetector.cpp
  TelltalesDetector.cpp
  VideoManager.cpp
  main.cpp
)

target_compile_options(DigitalClasterRecognition PRIVATE
    -Werror
    -Wall
    -Wextra
    -Wpedantic
    -Wcast-align
    -Wcast-qual
    -Wconversion
    -Wctor-dtor-privacy
    -Wenum-compare
    -Wfloat-equal
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wredundant-decls
    -Wsign-conversion
    -Wsign-promo)

target_link_libraries(DigitalClasterRecognition
     stdc++fs
     ${CMAKE_THREAD_LIBS_INIT}
     ${OpenCV_LIBS})

