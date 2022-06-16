#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "robotMotionControl.h"
#include "impPara/impPara.hpp"
#include <handler.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <dynamixel/dynamixel.h>

using namespace std;
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1

lcm::LCM Lcm;
robotCommand::robotCommand rc;
robotState::robotState rs;
MotionControl mc;
Matrix<float, 3, 1> force;
Matrix<float, 3, 1> tau;
ImpParaHandler ipHandle;
vector<int> ID = {3, 4, 5};
vector<float> start_pos = {0.0, 0.0, 0.0};
DxlAPI motors("/dev/ttyUSB0", 3000000, ID, 0);
Matrix<float, 1, 3> target_pos;
Matrix<float, 1, 3> target_vel;
Matrix<float, 1, 3> target_acc;
Matrix<float, 1, 3> xc_dotdot;
Matrix<float, 1, 3> xc_dot;
Matrix<float, 1, 3> xc;
vector<float> temp_pos;
float K = 1000;
float B = 30;
float M = 3;

void *robotCommandUpdate(void *data)
{
    
    while(0 == Lcm.handle());
}

void *robotStateUpdateSend(void *data)
{
    motors.setOperatingMode(3);  //3 position control; 0 current control
    motors.torqueEnable();
    motors.setPosition(start_pos);
    for(int i=0; i<3; i++)
    temp_pos.push_back(0.0);
    usleep(1e6);
    motors.getPosition();
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        mc.jointPstPos(i, j) = motors.present_position[j];
    }
    mc.updateFtsPstPos();
    target_pos = mc.ftsPstPos.row(0);
    target_vel << 0.0, 0.0, 0.0;
    target_acc << 0.0, 0.0, 0.0;
    xc_dot << 0.0, 0.0, 0.0;
    usleep(1e6);
    while(1)
    {

        motors.getTorque();
        motors.getPosition();
        motors.getVelocity();
        for(int i=0; i<4; i++)
        {
            for(int j=0; j<3; j++)
            {
                mc.jointPstPos(i, j) = motors.present_position[j];
                mc.jointPstVel(i, j) = motors.present_velocity[j];
            }

        }
        mc.updateFtsPstPos();
        mc.updateJacobians();
        mc.updateFtsPstVel();
        
        // mc.cmdFootPos = mc.ftsPstPos;
        // mc.inverseKinematics();
        for(int i=0; i<3; i++)
        tau(i,0) = motors.present_torque[i];
        force = mc.jacobian_vector[0].transpose().inverse() * tau;
        // xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
        xc_dotdot = 1/M*( -force.transpose() + B * (target_vel - mc.ftsPstVel.row(0)) +  K * (target_pos - mc.ftsPstPos.row(0))); //
        xc_dot =  mc.ftsPstVel.row(0) + xc_dotdot * 0.01;
        xc =  mc.ftsPstPos.row(0) + (xc_dot * 0.01);
        mc.cmdFootPos.row(0) = xc;
        mc.inverseKinematics();
        for(int i=0; i<3; i++)
        temp_pos[i] = mc.cmdJointPos(0,i);
        motors.setPosition(temp_pos);
        cout<<"xc_dotdot: "<<xc_dotdot<<"; xc_dot: "<<xc_dot<<"; xc: "<<xc<<endl;

        rs.F[0] = 0;
        rs.endPos[0] = 0;
        rs.endVel[0] = 0;
        Lcm.publish("ROBOTSTATE", &rs);
        // usleep(1e6);
    }
}

int main(int argc, char ** argv)
{
    Lcm.subscribe("IMPPARA", &ImpParaHandler::handleMessage, &ipHandle);

    pthread_t th1, th2;
	int ret;
	ret = pthread_create(&th1,NULL,robotCommandUpdate,NULL);
    if(ret != 0)
	{
		printf("create pthread1 error!\n");
		exit(1);
	}
    ret = pthread_create(&th2,NULL,robotStateUpdateSend,NULL);
    if(ret != 0)
	{
		printf("create pthread2 error!\n");
		exit(1);
	}
	
	pthread_join(th1, NULL);
    pthread_join(th2, NULL);
    while(1);

    
    
    return 0;
}
