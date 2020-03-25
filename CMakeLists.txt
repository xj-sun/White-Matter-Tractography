cmake_minimum_required(VERSION 2.8)

set (CMAKE_CXX_STANDARD 11)

project (VTKHW)

find_package (ITK REQUIRED)
include (${ITK_USE_FILE})

find_package (VTK REQUIRED)
include (${VTK_USE_FILE})

add_executable (VTKHW VTKHW.cpp)

include_directories (${CMAKE_SOURCE_DIR})

target_link_libraries (VTKHW ${ITK_LIBRARIES} ${VTK_LIBRARIES})
