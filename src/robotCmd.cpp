
#include <lcm/lcm-cpp.hpp>
#include "robotState/robotState.hpp"
#include "robotCommand/robotCommand.hpp"
#include "robotMotionControl.h"
#include "impPara/impPara.hpp"
#include <handler.h>
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
#include <js.h>

using namespace std;
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2
#define _JOYSTICK 1
#define MORTOR_ANGLE_AMP 40*3.14/180.0
#define loopRateCommandUpdate 100.0   //hz
#define loopRateStateUpdateSend 20.0   //hz
#define loopRateImpCtller 100.0   //hz
#define VELX 6.0 /1000   // mm  step length = VELX * timeForStancePhase        
#define TimePeriod 0.05
#define TimeForGaitPeriod 6

lcm::LCM Lcm;
robotCommand::robotCommand rc;
robotState::robotState rs;
impPara::impPara ip;
IMPControl imp;
ImpParaHandler ipHandle;
RobotStateHandler rsHandle;

vector<int> ID = {  
0,1,2,
3, 4, 5,
6,7,8
,9,10,11
};
DxlAPI motors("/dev/ttyUSB0", 1000000, ID, 1);  //3000000  cannot hold 6 legs ttyUSB0 ttyAMA0
vector<float> SetPos(12), SetTorque(12);

void *robotCommandUpdate(void *data)
{
    #ifdef _JOYSTICK
    int xbox_fd ;
    xbox_map_t map;
    int len;
    float vel = 0;
    float theta = 0;
    Vector<float, 3> TCV={ VELX, 0, 0 };// X, Y , alpha 

    memset(&map, 0, sizeof(xbox_map_t));
    xbox_fd = xbox_open("/dev/input/js0");
    map.lt = -32767;
    map.rt = -32767;
    while(1)
    {
        struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);
        len = xbox_map_read(xbox_fd, &map);
        if (len < 0)
        {
            usleep(10*1000);
            continue;
        }
        if (map.lx==0) theta = 0;
        else theta = float(atan2(-map.ly, map.lx) - 3.1416/2) / 5.0;
        vel = float(map.rt - map.lt) / 2000000.0;
        if(vel<0)
            vel= 0;
        TCV<<vel, 0.0, theta;
        imp.setCoMVel(TCV);
        
        //cout<<TCV.transpose()<<", "<<-map.ly<<", "<<map.lx<<", "<<map.lt<<", "<<map.rt<<endl;
        // cout<<"LO: "<<map.lo<<",RO: "<<map.ro<<",XBOX_AXIS_XX: "<<map.xx<<",XBOX_AXIS_YY: "<<map.yy<<",XBOX_BUTTON_LB: "<<map.lb<<",XBOX_BUTTON_RB: "<<map.rb<<endl;
        // cout<<"XBOX_BUTTON_A: "<<map.a<<"XBOX_BUTTON_B: "<<map.b<<"XBOX_BUTTON_x: "<<map.x<<"XBOX_BUTTON_y: "<<map.y<<endl;
        // cout<<map.start<<", "<<map.back<<", "<<map.home<<endl;
        fflush(stdout);
        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        //cout<<"thread1: "<<timeUse<<endl;
        usleep(1.0/loopRateCommandUpdate*1e6 - (double)(timeUse) - 10); // /* 1e4 / 1e6 = 0.01s */
    }
    #endif
    // while(0 == Lcm.handle());
}

void *robotStateUpdateSend(void *data)
{
    Matrix<float, 4, 2>TimeForStancePhase;
    Matrix<float, 4, 3> InitPos;
    Vector<float, 3> TCV={ VELX, 0, 0 };// X, Y , alpha 
    

    //motors initial
    motors.setOperatingMode(3);  //3 position control; 0 current control
    motors.torqueEnable();
    motors.getPosition();
    usleep(1e6);
#if(INIMODE==1)
    vector<float> init_Motor_angle(12);
    float float_init_Motor_angle[12];
    string2float("../include/init_Motor_angle.csv", float_init_Motor_angle);//Motor angle     d
    //cout<<"____________"<<endl;
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            float_init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j] * 3.1416/180; //to rad
            init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j];      //vector
            imp.jointCmdPos(i,j) = float_init_Motor_angle[i*3+j];            //imp.forwardKinematics
            //cout<<init_Motor_angle[i*3+j]<<endl;
        }
    imp.forwardKinematics(0);
    imp.setInitPos(imp.legCmdPos);        //legCmdPos
    cout<<"legCmdPos:\n"<<imp.legCmdPos<<endl ;

    motors.setPosition(init_Motor_angle);
#endif    
    
    //      imp initial
    // TimeForStancePhase<< 0,                       TimeForGaitPeriod/2.0,     // diagonal
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      0,                       TimeForGaitPeriod/2.0;
    TimeForStancePhase<< TimeForGaitPeriod/4.0 *3,          TimeForGaitPeriod/4.0 *2,   // tripod
                         TimeForGaitPeriod/4.0,             TimeForGaitPeriod,
                         TimeForGaitPeriod - TimePeriod,    TimeForGaitPeriod/4.0 *3,
                         TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0;
    imp.setPhase(TimePeriod, TimeForGaitPeriod, TimeForStancePhase);

#if(INIMODE==2)
    float  float_initPos[12];
    string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            InitPos(i, j) = float_initPos[i*3+j];
            //cout<<InitPos(i, j)<<endl;
        }
    imp.setInitPos(InitPos);
#endif

    imp.setCoMVel(TCV);
    imp.inverseKinematics(imp.legCmdPos);
#if(INIMODE==2)  
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
           
            SetPos[i*3+j] = imp.jointCmdPos(i,j);
            cout<<"SetPos:"<<SetPos[i*3+j] <<endl;
            if( isnanf(SetPos[i*3+j]) )
                SetPos[i*3+j] = 0;
        }
    motors.setPosition(SetPos);     
#endif


    // ofstream outputfile;
    // outputfile.open("../include/output.csv");
    // for(int i=0; i<4; i++)
    // {
    //     for(int j=0;j<3;j++)
    //         outputfile<<imp.jointCmdPos(i,j)<<" ";
    //     outputfile<<endl;
    // }
    // outputfile.close();


    imp.target_pos = imp.legCmdPos;
    imp.initFlag = 1;
    usleep(1e5);
    while(1)
    {
        struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);

        //If stay static, annotate below one line.
        imp.nextStep();//
        imp.impParaDeliver();
        // cout<<"legCmdPos:\n"<<imp.legCmdPos<<endl;

        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1e4)
            usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 10); 
        else
            cout<<"TimeRobotStateUpdateSend: "<<timeUse<<endl;

    }
    
}

void *runImpCtller(void *data)
{
    struct timeval startTime,endTime;
    double timeUse;
    int run_times=0;    // for debugging

    while(imp.initFlag == 0) //wait for initial
        usleep(1e2);
    usleep(1e5);
    while (1)
    {
        gettimeofday(&startTime,NULL);
        /* get motors data  */
        while( motors.getTorque()==false || motors.getPosition()==false || motors.getVelocity()==false );
        /* update the data IMP need */
        imp.updatejointPresPos(motors.present_position);         
        imp.updatejointPresVel(motors.present_velocity);
        imp.forwardKinematics(1);
        imp.updateJacobians();
        imp.updateFtsPresVel();

        imp.updateFtsPresForce(motors.present_torque);  

        // for(int i=0; i<4; i++)  
        // {
        //     for(int j=0;j<3;j++)
        //     {
        //         // ip.target_pos[i*3 + j] = imp.legCmdPos(i,j);
        //         // ip.target_vel[i*3 + j] = ;
        //         // ip.target_acc[i*3 + j] = ;
        //         // ip.target_force[i*3 + j] = ;
        //         ip.force[i*3 + j] = imp.force(j,i);
        //     }
        //     ip.stepFlag[i] = (int) imp.stepFlag[i];
        //     ip.timePresentForSwing[i] = imp.timePresentForSwing(i);
        // }
        // Lcm.publish("IMPTAR", &ip);

        // ip.force[11] = imp.force(2,3);
        // ip.xc[9] = imp.legPresVel(3, 2);
        // ip.xc[10] = imp.legPresPos(3,2);
        // ip.xc[11] = imp.xc(3, 2);
        // ip.stepFlag[3] = (int)imp.stepFlag[3];
        // Lcm.publish("IMPTAR", &ip);

        // imp.inverseKinematics(imp.target_pos); //    within impCtller
        imp.target_pos<<imp.initFootPos;
        imp.impCtller(1);   
        imp.inverseKinematics(imp.xc);   //    Admittance control
        
        // cout<<"target_pos: \n"<<imp.target_pos<<endl;
        cout<<"legPresPos: \n"<<imp.legPresPos<<"; \nxc: \n"<<imp.xc<<endl;
        cout<<"force:"<<endl<<imp.force.transpose()<<endl;
        // cout<<"xc_dotdot: \n"<<imp.xc_dotdot<<"; \nxc_dot: \n"<<imp.xc_dot<<"; \nxc: \n"<<imp.xc<<endl;
        // cout<<"legPresPos: \n"<<imp.legPresPos<<endl;
        cout<<endl;

        /*      Admittance control      */
        for(int i=0; i<4; i++)  
            for(int j=0;j<3;j++)
            {
                if( isnanf(imp.jointCmdPos(i,j)) )            
                {
                    imp.jointCmdPos(i,j) = SetPos[i*3+j];   // last
                    cout<<"-------------motor_angle_"<<i*3+j<<" NAN-----------"<<endl;
                    // cout<<"target_pos: \n"<<imp.target_pos<<"; \nxc: \n"<<imp.xc<<endl;
                    exit(0);
                }
                else
                {
                    if(imp.jointCmdPos(i,j) - SetPos[i*3+j] > MORTOR_ANGLE_AMP)
                    {
                        SetPos[i*3+j] += MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" +MAX-----------"<<endl;
                    }
                    else if(imp.jointCmdPos(i,j) - SetPos[i*3+j] < -MORTOR_ANGLE_AMP)
                    {
                        SetPos[i*3+j] -= MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" -MAX-----------"<<endl;
                    }
                    else 
                        SetPos[i*3+j] = imp.jointCmdPos(i,j);   // now
                }
                //cout<<"motor_angle_"<<i*3+j<<": "<<SetPos[i*3+j]<<"  ";
            }   
        motors.setPosition(SetPos); 

        /*      Impedance control      */
        // for(int i=0; i<4; i++)  
        //     for(int j=0;j<3;j++)
        //         SetTorque[i*3+j] = imp.target_torque(j,i);
        // motors.setTorque(SetTorque); 

        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1e4)
            usleep(1.0/loopRateImpCtller*1e6 - (double)(timeUse) - 10); 
        else
            cout<<"timeImpCtller: "<<timeUse<<endl;
        
        // run_times++;
        // if(run_times++ > 10)
        // {
        //     motors.torqueDisable();
        //     exit(0);
        // }
    }
}

int main(int argc, char ** argv)
{
    // Lcm.subscribe("ROBOTCOMMAND", &RobotStateHandler::handleMessage, &rsHandle);

    pthread_t th1, th2, th3;
	int ret;
	// ret = pthread_create(&th1,NULL,robotCommandUpdate,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread1 error!\n");
	// 	exit(1);
	// }
    ret = pthread_create(&th2,NULL,robotStateUpdateSend,NULL);
    if(ret != 0)
	{
		printf("create pthread2 error!\n");
		exit(1);
	}
    ret = pthread_create(&th3,NULL,runImpCtller,NULL);
    if(ret != 0)
	{
		printf("create pthread3 error!\n");
		exit(1);
	}
    
	
	//pthread_join(th1, NULL);
    pthread_join(th2, NULL);
    pthread_join(th3, NULL);
    while(1);

    
    
    return 0;
}
