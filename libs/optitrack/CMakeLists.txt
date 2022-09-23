project(OPTITRACK)

add_library(
    optitrack STATIC
    include/optitrack.hpp
    include/optitrack_channels.h
    src/optitrack.cpp
    )

target_include_directories(optitrack PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src") 
target_include_directories(optitrack PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")