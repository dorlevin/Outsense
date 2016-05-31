cmake_minimum_required (VERSION 2.8)
project (outsense)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories (
                ${PROJECT_SOURCE_DIR}/include
)

find_package(OpenCV REQUIRED)


if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()


add_executable (outsense outsense.cpp)
target_link_libraries (outsense mraa ${OpenCV_LIBS})
