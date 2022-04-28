cmake_minimum_required(VERSION 2.9)
project( useEigen )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb" )

include_directories( "/usr/local/include/" )
link_directories("/usr/local/lib")

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(stateEst src/stateEst.cpp)
target_link_libraries(stateEst lcm pthread)

add_executable(robotCmd src/robotCmd.cpp)
target_link_libraries(robotCmd lcm pthread)

add_executable(impCtl src/impCtl.cpp)
target_link_libraries(impCtl lcm pthread)
