#!/bin/bash
# cd home/pi/Multiprocessing-control-structure-based-on-LCM/build
cd build
cmake ..
make
./impCtl &
# ./stateEst &
./robotCmd
