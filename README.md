# Multiprocessing-control-structure-based-on-LCM

## File structure 
```
.
├── CMakeLists.txt
├── include/
│   ├── handler.hpp  # Handle to get the transmitting data
│   ├── impPara/
│   │   └── impPara.hpp  # The parameter for impedance control
│   ├── impPara.lcm
│   ├── robotCommand/
│   │   └── robotCommand.hpp  # The command for the robot actuator 
│   ├── robotCommand.lcm
│   ├── robotState/
│   │   └── robotState.hpp  # The data of robot state
│   └── robotState.lcm
├── run.sh
└── src/
    ├── impCtl.cpp  # impedance control program
    └── stateEst.cpp  # state estimating program
```

## Requirement

1. LCM
2. Eigen 
