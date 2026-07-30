cmake_minimum_required(VERSION 3.22)
project(planescreen)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_PREFIX_PATH "~/Qt/6.9.0/gcc_64")

find_package(Qt6 COMPONENTS
  Core
  Widgets
  Network
  REQUIRED)

add_executable(planescreen main.cpp
        home.cpp
        home.h
        blink.cpp
        blink.h
        plane_targets.h
        targetInfo.cpp
        targetInfo.h
)
target_link_libraries(planescreen
  Qt::Core
  Qt::Widgets
  Qt::Network
)

