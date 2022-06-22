#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "robotMotionControl.h"
#include "impPara/impPara.hpp"
#include <handler.hpp>
#include <iostream>
#include <dynamixel/dynamixel.h>
#include <robotCmd.h>

using namespace std;
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1

lcm::LCM Lcm;
robotCommand::robotCommand rc;
robotState::robotState rs;
MotionControl mc;
IMPControl imp;

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

IMPControl::IMPControl()
{
    K<<1000,1000,1000,1000;
    B<<30,30,30,30;
    M<<3,3,3,3;
    xc_dotdot.setZero();
    xc_dot.setZero();
    xc.setZero();
    target_pos.setZero();
    target_vel.setZero();
    target_acc.setZero();
}
/*
impdeliver();
imp.impCtller( );
mc.legCmdPos = imp.xc;
mc.inverseKinematics();
for(int i=0; i<3; i++)
    for(int j=0;j<4;j++)
        temp_pos[i*4+j] = mc.joinCmdPos(i,j);
motors.setPosition(temp_pos);
*/
void impdeliver()
{
    Matrix<float, 3, 4> temp;
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = motors.present_torque[i*4+j];
    for (int i=0; i<4; i++)
        imp.force.col(i) = mc.jacobian_vector[i].transpose().inverse() * temp.col(i);
    imp.target_pos = mc.legCmdPos;
    //imp.target_vel = 0;
    //imp.target_acc = 0; //
}
void IMPControl::impCtller()
{
    // xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    xc_dotdot =  target_acc + M.cwiseInverse() * ( -force.transpose() + B.cwiseProduct(target_vel - mc.ftsPstVel) +  K.cwiseProduct(target_pos - mc.ftsPstPos)); //
    xc_dot =  mc.ftsPstVel + xc_dotdot * 0.01;
    xc =  mc.ftsPstPos + (xc_dot * 0.01);
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
    motors.setPosition(start_pos);
    for(int i=0; i<12; i++)
        temp_pos.push_back(0.0);
    usleep(1e6);
    motors.getPosition();
    //mc initial
    float timePeriod = 0.01;
    float timeForGaitPeriod = 0.49;
    Matrix<float, 4, 2> timeForStancePhase = { 0, 0.24, 0.25, 0.49, 0.25, 0.49, 0, 0.24};
    Matrix<float, 4, 3> initPos = {3.0, 0.0, -225.83, 3.0, 0.0, -225.83, -20.0, 0.0, -243.83, -20.0, 0.0, -243.83};
    mc.setPhase(timePeriod, timeForGaitPeriod, timeForStancePhase);
    mc.setInitPos(initPos);

    mc.updateJointPstPos(motors.present_position);
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
        
        // for(int i=0; i<3; i++)
        //     tau(i,0) = motors.present_torque[i];
        // force = mc.jacobian_vector[0].transpose().inverse() * tau;
        // // xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
        // xc_dotdot = 1/M*( -force.transpose() + B * (target_vel - mc.ftsPstVel.row(0)) +  K * (target_pos - mc.ftsPstPos.row(0))); //
        // xc_dot =  mc.ftsPstVel.row(0) + xc_dotdot * 0.01;
        // xc =  mc.ftsPstPos.row(0) + (xc_dot * 0.01);

        // mc.legCmdPos.row(0) = xc;
        // mc.inverseKinematics();

        // for(int i=0; i<3; i++)
        //     temp_pos[i] = mc.joinCmdPos(0,i);
        // motors.setPosition(temp_pos);

        mc.nextStep();

        impdeliver();
        imp.impCtller( );
        mc.legCmdPos = imp.xc;
        mc.inverseKinematics();
        for(int i=0; i<3; i++)
            for(int j=0;j<4;j++)
                temp_pos[i*4+j] = mc.joinCmdPos(i,j);
        motors.setPosition(temp_pos);     

        cout<<"xc_dotdot: "<<xc_dotdot<<"; xc_dot: "<<xc_dot<<"; xc: "<<xc<<endl;

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
