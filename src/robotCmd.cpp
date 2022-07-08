#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "robotMotionControl.h"
#include "impPara/impPara.hpp"
#include <handler.hpp>
#include <iostream>
#include <fstream>
#include <string.h>
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
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2

lcm::LCM Lcm;
robotCommand::robotCommand rc;
robotState::robotState rs;
//MotionControl mc;
IMPControl imp;

Matrix<float, 3, 1> force;
Matrix<float, 3, 1> tau;
ImpParaHandler ipHandle;
vector<int> ID = {
0,1,2//,
//3, 4, 5,
// 6,7,8
// ,9,10,11
};
vector<float> start_pos;

DxlAPI motors("/dev/ttyAMA0", 3000000, ID, 1);  //3000000  cannot hold 6 legs
Matrix<float, 1, 3> target_pos;
Matrix<float, 1, 3> target_vel;
Matrix<float, 1, 3> target_acc;
Matrix<float, 1, 3> xc_dotdot;
Matrix<float, 1, 3> xc_dot;
Matrix<float, 1, 3> xc;
vector<float> temp_pos,temp_pos2;
float K = 1000;
float B = 30;
float M = 3;

void string2float(string add, float* dest)
{
    char data_char[1000],*char_float;
    const char *a=",";
    int i=0;
    ifstream inidata;

    inidata.open(add);
    if (inidata)    cout<<"file open Successful"<<endl;
    else    cout<<"file open FAIL"<<endl;
    inidata.read(data_char,1000);
    char_float=strtok(data_char, a);
    while(char_float!=NULL)
    {        
        dest[i++] = stof(char_float);
        // printf ("%s",char_float);
        // cout<<'|'<<(*dest)[i-1]<<endl;
        char_float=strtok(NULL, a);
    }
    inidata.close();
}

void *robotCommandUpdate(void *data)
{
    
    while(0 == Lcm.handle());
}

void *robotStateUpdateSend(void *data)
{
    //motors initial
    motors.setOperatingMode(3);  //3 position control; 0 current control
    motors.torqueEnable();
    motors.getPosition();
    usleep(1e6);
    for(int i=0; i<12; i++)
    {
        temp_pos.push_back(0.0);
        temp_pos2.push_back(0.0);
        //cout<<motors.present_position[i]<<endl;
    }
#if(INIMODE==1)
    float float_start_pos[12];
    string2float("../include/start_pos.csv", float_start_pos);//Motor angle     rad
    for(int i=0;i<12;i++)
    {
        start_pos[i] = float_start_pos[i];
        cout<<start_pos[i]<<endl;
    }
    motors.setPosition(start_pos);
#endif    
    
    
    //imp initial
    float timePeriod = 0.01;
    float timeForGaitPeriod = 0.49;
    Matrix<float, 4, 2> timeForStancePhase ;
    Matrix<float, 4, 3> initPos;
    Vector<float, 3> tCV={1, 0, 0 };// X, Y , alpha 
    timeForStancePhase << 0, 0.24, 0.25, 0.49, 0.25, 0.49, 0, 0.24;
    imp.setPhase(timePeriod, timeForGaitPeriod, timeForStancePhase);
    //initPos << 3.0, 0.0, -225.83, 3.0, 0.0, -225.83, -20.0, 0.0, -243.83, -20.0, 0.0, -243.83; //xyz
#if(INIMODE==2)
    float  float_initPos[12];
    string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            initPos(i, j) = float_initPos[i*3+j];
            cout<<initPos(i, j)<<endl;
        }
#endif
    imp.setInitPos(initPos);
    imp.setCoMVel(tCV);
    imp.inverseKinematics();
#if(INIMODE==2)  
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            temp_pos[i*3+j] = imp.joinCmdPos(i,j);
            cout<<"temp_pos:"<<temp_pos[i*3+j] <<endl;
        }
    //motors.setPosition(temp_pos);     
#endif


    // imp.updateJointPstPos(motors.present_position);
    // imp.updateFtsPstPos();
    // target_pos = imp.ftsPstPos.row(0);
    usleep(1e6);
    while(1)
    {
        /*
        // get motors data
        motors.getTorque();
        motors.getPosition();
        motors.getVelocity();
        // update the data IMP need
        imp.updateJointPstPos(motors.present_position);
        imp.updateJointPstVel(motors.present_velocity);
        imp.updateFtsPstPos();
        imp.updateJacobians();
        imp.updateFtsPstVel();
        
        // for(int i=0; i<3; i++)
        //     tau(i,0) = motors.present_torque[i];
        // force = imp.jacobian_vector[0].transpose().inverse() * tau;
        // // xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
        // xc_dotdot = 1/M*( -force.transpose() + B * (target_vel - imp.ftsPstVel.row(0)) +  K * (target_pos - imp.ftsPstPos.row(0))); //
        // xc_dot =  imp.ftsPstVel.row(0) + xc_dotdot * 0.01;
        // xc =  imp.ftsPstPos.row(0) + (xc_dot * 0.01);

        // imp.legCmdPos.row(0) = xc;
        // imp.inverseKinematics();

        // for(int i=0; i<3; i++)
        //     temp_pos[i] = imp.joinCmdPos(0,i);
        // motors.setPosition(temp_pos);

        imp.nextStep();

        imp.impdeliver(motors.present_torque);  
        imp.impCtller();
        imp.inverseKinematics();
        for(int i=0; i<4; i++)
            for(int j=0;j<3;j++)
                temp_pos[i*3+j] = imp.joinCmdPos(i,j);
        //motors.setPosition(temp_pos);     

        //cout<<"xc_dotdot: "<<imp.xc_dotdot<<"; xc_dot: "<<imp.xc_dot<<"; xc: "<<imp.xc<<endl;
        */

       //test time
    //    struct timeval startTime, endTime1,endTime2;
    //    double timeUse1,timeUse2;
    //    gettimeofday(&startTime,NULL);

    //     motors.getTorque();
    //     motors.getPosition();
    //     motors.getVelocity();
    //     gettimeofday(&endTime1,NULL);
    //     timeUse1 = 1000000*(endTime1.tv_sec - startTime.tv_sec) + endTime1.tv_usec - startTime.tv_usec;
    //     for(int i=0; i<4; i++)
    //         for(int j=0;j<3;j++)
    //             temp_pos[i*3+j] = 0;
    //     motors.setPosition(temp_pos);     
    //     gettimeofday(&endTime2,NULL);
    //     timeUse2 = 1000000*(endTime2.tv_sec - startTime.tv_sec) + endTime2.tv_usec - startTime.tv_usec;
    //     cout<<"timeUse1:"<<timeUse1<<"timeUse2:"<<timeUse2<<endl;

    //     rs.F[0] = 0;
    //     rs.endPos[0] = 0;
    //     rs.endVel[0] = 0;
    //     Lcm.publish("ROBOTSTATE", &rs);

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
