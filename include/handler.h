#ifndef handler_H
#define handler_H

#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "impPara/impPara.hpp"
#include "robotCommand/robotCommand.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>
#include "robotMotionControl.h"

class ImpParaHandler : public IMPControl
{
    public:
        void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const impPara::impPara* msg);

};

class RobotStateHandler 
{
    public:

        float F[12];
        float endPos[12];
        float endVel[12];
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const robotState::robotState* msg);
};

class RobotCommandHandler 
{
    public:

        float targetEndPos[12];
        bool robotRunEnable;
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const robotCommand::robotCommand* msg);
};

#endif