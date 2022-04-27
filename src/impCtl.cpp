#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
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
    robotCommand::robotCommand rc;
    ImpParaHandler ipHandle;
    RobotStateHandler rsHandle;
    lcm.subscribe("IMPPARA", &ImpParaHandler::handleMessage, &ipHandle);
    lcm.subscribe("ROBOTSTATE", &RobotStateHandler::handleMessage, &rsHandle);

    for(int j=0; j<100; j++)
    {
        lcm.handle();
        for(int i=0; i<12; i++)
        {
            rc.targetEndPos[i] = j;
        }
        lcm.publish("ROBOTCOMMAND", &rc);
        usleep(1e6);
    }

    
    return 0;
}
