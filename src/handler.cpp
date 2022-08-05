#include "handler.h"

void ImpParaHandler::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const impPara::impPara* msg)
{
    for(int i=0; i<4; i++)  
    {
        for(int j=0;j<3;j++)
        {
            force(j, i) = msg->force[i + j*4];
            target_pos(i, j) = msg->target_pos[i*3 + j];
            target_vel(i, j) = msg->target_vel[i*3 + j];
            target_acc(i, j) = msg->target_acc[i*3 + j];
            target_force(i, j) = msg->target_force[i*3 + j];
            xc(i, j) = msg->xc[i*3 + j];
        }
        stepFlag(i) = msg->stepFlag[i];
        timePresentForSwing(i) = msg->timePresentForSwing[i];
    }
}

void RobotStateHandler::handleMessage(const lcm::ReceiveBuffer* rbuf,
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

void RobotCommandHandler::handleMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan, 
        const robotCommand::robotCommand* msg)
{
    for(int i=0; i<12; i++)
    {
        targetEndPos[i] = msg->targetEndPos[i];
    }
    robotRunEnable = msg->robotRunEnable;
}