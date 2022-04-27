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

class ImpParaHandler 
{
    public:
        ~ImpParaHandler() {}
        float K[12];
        float D[12];
        float P[12];
        float V[12];
        float Fr[12];
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const impPara::impPara* msg)
        {
            for(int i=0; i<12; i++)
            {
                K[i] = msg->K[i];
                D[i] = msg->D[i];
                P[i] = msg->P[i];
                V[i] = msg->V[i];
                Fr[i] = msg->Fr[i];
            }
        }
};

class RobotStateHandler 
{
    public:
        ~RobotStateHandler() {}
        float F[12];
        float endPos[12];
        float endVel[12];
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const robotState::robotState* msg)
        {
            for(int i=0; i<12; i++)
            {
                F[i] = msg->F[i];
                endPos[i] = msg->endPos[i];
                endVel[i] = msg->endVel[i];
            }
        }
};

class RobotCommandHandler 
{
    public:
        ~RobotCommandHandler() {}
        float targetEndPos[12];
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const robotCommand::robotCommand* msg)
        {
            for(int i=0; i<12; i++)
            {
                targetEndPos[i] = msg->targetEndPos[i];
            }
        }
};
