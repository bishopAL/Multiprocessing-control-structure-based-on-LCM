#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "impPara/impPara.hpp"
#include <handler.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace std;

int main(int argc, char ** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    robotState::robotState rs;
    impPara::impPara ip;
    RobotStateHandler rsHandle;
    lcm.subscribe("ROBOTSTATE", &RobotStateHandler::handleMessage, &rsHandle);

    for(int j=0; j<100; j++)
    {
        lcm.handle();
        for(int i=0; i<12; i++)
        {
            ip.D[i] = j;
            ip.K[i] = j;
            ip.P[i] = j;
            ip.V[i] = j;
            ip.Fr[i] = j;
        }
        lcm.publish("IMPPARA", &ip);
        usleep(1e6);
    }

    
    return 0;
}
