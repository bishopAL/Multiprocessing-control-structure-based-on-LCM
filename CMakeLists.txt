cmake_minimum_required(VERSION 2.9)
project( useEigen )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb" )

include_directories( "/usr/local/include/" )
link_directories("/usr/local/lib")
include_directories(${PROJECT_SOURCE_DIR}/include)

message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME} ")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(dynamixel STATIC ${PROJECT_SOURCE_DIR}/src/dynamixel.cpp)
target_include_directories(dynamixel
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_library(RobotMChandler STATIC ${PROJECT_SOURCE_DIR}/src/handler.cpp src/robotMotionControl.cpp)
target_include_directories(RobotMChandler
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

if(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
set( DYNAMIXELLIB "dxl_mac_cpp")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
set( DYNAMIXELLIB "dxl_x64_cpp")
else()
set( DYNAMIXELLIB "dxl_sbc_cpp")
endif()

add_executable(stateEst src/stateEst.cpp)
target_link_libraries(stateEst lcm pthread RobotMChandler)

add_executable(robotCmd src/robotCmd.cpp)   
target_link_libraries(robotCmd lcm pthread dynamixel ${DYNAMIXELLIB} RobotMChandler)   # lib must in order: dynamixel dxl_sbc_cpp 

add_executable(impCtl src/impCtl.cpp)
target_link_libraries(impCtl lcm pthread RobotMChandler)

