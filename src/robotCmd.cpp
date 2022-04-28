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
#include <pthread.h>

using namespace std;
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1

lcm::LCM Lcm;
robotCommand::robotCommand rc;
robotState::robotState rs;
ImpParaHandler ipHandle;

void *robotCommandUpdate(void *data)
{
    while(0 == Lcm.handle());
}

void *robotStateUpdateSend(void *data)
{
    
    for(int j=0; j<100; j++)
    {
        rs.F[0] = j;
        rs.endPos[0] = j;
        rs.endVel[0] = j;
        Lcm.publish("ROBOTSTATE", &rs);
        usleep(1e6);
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
