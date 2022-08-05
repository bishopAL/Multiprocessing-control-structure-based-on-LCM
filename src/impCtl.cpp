#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "impPara/impPara.hpp"
#include "handler.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include "robotMotionControl.h"

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
#define MORTOR_ANGLE_AMP 20*3.14/180.0
#define loopRateImpCtller 100

lcm::LCM Lcm;
robotCommand::robotCommand rc;
ImpParaHandler ipHandle;
RobotStateHandler rsHandle;

void *paraUpdate(void *data)
{
    ofstream outputfile;
    outputfile.open("../include/output.csv");
    
    while(1)
    {
        if(0 == Lcm.handle())
        {
            // for(int i=0; i<3; i++)
            // {
            //     for(int j=0; j<4; j++)
            //         outputfile<<ipHandle.force(i,j)<<",";
            // }
            outputfile<<ipHandle.force(2,3)<<",";
            outputfile<<ipHandle.xc(3,2);
            outputfile<<"\r\n";
        }
        usleep(1e4);
    }
    outputfile.clear();
    outputfile.close();
}

void *impCtller(void *data)
{
    // for(int j=0; j<100; j++)
    // {
    //     for(int i=0; i<12; i++)
    //     {
    //         rc.targetEndPos[i] = j;
    //     }
    //     // Lcm.publish("ROBOTCOMMAND", &rc);
    //     usleep(1e6);
    // }
}

int main(int argc, char ** argv)
{
    Lcm.subscribe("IMPTAR", &ImpParaHandler::handleMessage, &ipHandle);
    Lcm.subscribe("ROBOTSTATE", &RobotStateHandler::handleMessage, &rsHandle);

    pthread_t th1, th2;
	int ret;
	ret = pthread_create(&th1,NULL,paraUpdate,NULL);
    if(ret != 0)
	{
		printf("create pthread1 error!\n");
		exit(1);
	}
    ret = pthread_create(&th2,NULL,impCtller,NULL);
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
