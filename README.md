# Multiprocessing-control-structure-based-on-LCM

## Requirement
1. Software setup, including [LCM](http://lcm-proj.github.io), [Eigen](eigen.tuxfamily.org/) and [Dynamixel API](https://github.com/bishopAL/GeRot/tree/master/API/dynamixel_cpp%20Ver2.0). Please install them correctly. The Eigen could be installed in `/usr/local/include`.
2. Hardware setup. If the communication port is `/dev/ttyAMA0`, please check this [blog](https://blog.csdn.net/sinat_37939098/article/details/119344651?spm=1001.2014.3001.5502) to get how to setup the port. Make sure the communication is stable. If the communication port is `/dev/ttyUSB0` with U2D2, please check this [readme](https://github.com/bishopAL/GeRot/blob/master/README.md), the *Environmental Configuration* is the tutorial you need.
3. Optional hardware, including joystick and IMU. If you don't need them, you may set the corresponding `pthread_t` disable in *robotCmd.cpp*.

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
│   ├── imp_parameter.csv   # The parameters K,B,M of impCtller in pahse of stance/ swing/ detach/ attach.
│   ├── init_Motor_angle.csv
│   ├── initPos.csv
│   ├── js.h    # The joystick of XBOX to contrl the direction of robot
│   ├── output.csv
│   ├── robotCommand
│   │   └── robotCommand.hpp  # The command for the robot actuator 
│   ├── robotCommand.lcm
│   ├── robotMotionControl.h  # The class to access motion control
│   ├── robotState
│   │   └── robotState.hpp  # The data of robot state
│   └── robotState.lcm
├── lib
│   ├── libdynamixel.a
│   ├── libdynamixel_lib.a
│   └── libRobotMChandler.a
├── README.md
├── run.sh
└── src
    ├── dynamixel.cpp
    ├── handler.cpp
    ├── impCtl.cpp
    ├── robotCmd.cpp
    ├── robotMotionControl.cpp
    └── stateEst.cpp
```

