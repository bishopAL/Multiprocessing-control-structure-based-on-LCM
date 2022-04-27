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
#include <pthread.h>

using namespace std;
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1

lcm::LCM Lcm;
robotCommand::robotCommand rc;
impPara::impPara ip;
ImpParaHandler ipHandle;
RobotStateHandler rsHandle;

void *robotStateUpdate(void *data)
{
    while(0 == Lcm.handle());
}

void *stateEst(void *data)
{
    for(int j=0; j<100; j++)
    {
        for(int i=0; i<12; i++)
        {
            ip.D[i] = j;
            ip.K[i] = j;
            ip.P[i] = j;
            ip.V[i] = j;
            ip.Fr[i] = j;
        }
        Lcm.publish("IMPPARA", &ip);
        usleep(1e6);
    }
}

int main(int argc, char ** argv)
{
    Lcm.subscribe("ROBOTSTATE", &RobotStateHandler::handleMessage, &rsHandle);

    pthread_t th1, th2;
	int ret;
	ret = pthread_create(&th1,NULL,robotStateUpdate,NULL);
    if(ret != 0)
	{
		printf("create pthread1 error!\n");
		exit(1);
	}
    ret = pthread_create(&th2,NULL,stateEst,NULL);
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
