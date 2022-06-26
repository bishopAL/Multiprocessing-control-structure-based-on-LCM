cmake_minimum_required(VERSION 2.9)
project( useEigen )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb" )

include_directories( "/usr/local/include/" )
link_directories("/usr/local/lib")
include_directories(${PROJECT_SOURCE_DIR}/include)

message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME} ")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
if(CMAKE_HOST_SYSTEM_NAME MATCHES "APPLE")
set( DYNAMIXELLIB "dxl_mac_cpp")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
set( DYNAMIXELLIB "dxl_x64_cpp")
else()
set( DYNAMIXELLIB "dxl_sbc_cpp")
endif()
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(dynamixel_lib STATIC ${PROJECT_SOURCE_DIR}/src/dynamixel.cpp)
target_include_directories(dynamixel_lib
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(stateEst src/stateEst.cpp)
target_link_libraries(stateEst lcm pthread)

add_executable(robotCmd src/robotCmd.cpp src/robotMotionControl.cpp )   #src/dynamixel.cpp
target_link_libraries(robotCmd lcm pthread dynamixel_lib ${DYNAMIXELLIB})   #  must in order: dynamixel_lib dxl_sbc_cpp
#dxl_x64_cpp

add_executable(impCtl src/impCtl.cpp)
target_link_libraries(impCtl lcm pthread)

