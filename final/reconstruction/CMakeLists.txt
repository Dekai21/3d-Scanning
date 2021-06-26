cmake_minimum_required( VERSION 2.8 )
project(reconstruction)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# find_package( OpenCV 3.1 REQUIRED )
find_package( OpenCV 3.2 REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    "/usr/include/eigen3/"
)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

set(HEADERS 
    "include.h"
)

set(SOURCES
    main.cpp
    orb.cpp
)

add_executable( main ${HEADERS} ${SOURCES}  )
target_link_libraries( main ${OpenCV_LIBS} )