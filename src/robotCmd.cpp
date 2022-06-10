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
#include <dynamixel.h>

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
vector<int> ID = {0, 1, 2};
vector<float> start_pos = {0.0, 0.0, 0.0};
DxlAPI motors("/dev/ttyUSB0", 3000000, ID, 0);


void *robotCommandUpdate(void *data)
{
    
    while(0 == Lcm.handle());
}

void *robotStateUpdateSend(void *data)
{
    motors.setOperatingMode(3);  //3 position control; 0 current control
    motors.torqueEnable();
    motors.setPosition(start_pos);
    
    while(1)
    {
        motors.getTorque();
        motors.getPosition();
        for(int i=0; i<4; i++)
        {
            for(int j=0; j<3; j++)
            mc.jointPstPos(i, j) = motors.present_position[j];
        }
        mc.forwardKinematics();
        mc.updateJacobians();
        // mc.cmdFootPos = mc.ftsPstPos;
        // mc.inverseKinematics();
        for(int i=0; i<3; i++)
        tau(i,0) = motors.present_current[i];
        force = mc.jacobian_vector[0].transpose().inverse() * tau;
        cout<<force<<endl;
        rs.F[0] = 0;
        rs.endPos[0] = 0;
        rs.endVel[0] = 0;
        Lcm.publish("ROBOTSTATE", &rs);
        usleep(1e3);
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
