# Multiprocessing-control-structure-based-on-LCM

## File structure 
```
.
├── CMakeLists.txt
├── include
│   ├── dynamixel
│   │   ├── dynamixel.h  # motor API
│   │   ├── dynamixel_sdk.h
│   │   ├── group_bulk_read.h
│   │   ├── group_bulk_write.h
│   │   ├── group_sync_read.h
│   │   ├── group_sync_write.h
│   │   ├── packet_handler.h
│   │   ├── port_handler_arduino.h
│   │   ├── port_handler.h
│   │   ├── port_handler_linux.h
│   │   ├── port_handler_mac.h
│   │   ├── port_handler_windows.h
│   │   ├── protocol1_packet_handler.h
│   │   └── protocol2_packet_handler.h
│   ├── handler.hpp  # Handle to get the transmitting data
│   ├── impPara
│   │   └── impPara.hpp  # The parameter for impedance control
│   ├── impPara.lcm
│   ├── robotCommand
│   │   └── robotCommand.hpp  # The command for the robot actuator 
│   ├── robotCommand.lcm
│   ├── robotMotionControl.h  # The class to access motion control
│   ├── robotState
│   │   └── robotState.hpp  # The data of robot state
│   └── robotState.lcm
├── README.md
├── run.sh
└── src
    ├── dynamixel.cpp
    ├── impCtl.cpp
    ├── robotCmd.cpp
    ├── robotMotionControl.cpp
    └── stateEst.cpp
```

## Requirement

1. [LCM](http://lcm-proj.github.io)
2. [Eigen](eigen.tuxfamily.org/)
3. [Dynamixel API](https://github.com/bishopAL/GeRot/tree/master/API/dynamixel_cpp%20Ver2.0)
4. [DxlAPI Interface]  If use /dev/ttyAMA0, go to (https://blog.csdn.net/sinat_37939098/article/details/119344651?spm=1001.2014.3001.5502)
