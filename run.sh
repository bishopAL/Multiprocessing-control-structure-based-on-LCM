#!/bin/bash
cd build
cmake ..
make
./impCtl &
./stateEst &
