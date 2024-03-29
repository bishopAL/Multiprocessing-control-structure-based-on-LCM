#ifndef robotMotionControl_H
#define robotMotionControl_H

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <fstream>

using namespace std;
using namespace Eigen;

class MotionControl
{
    public:
        float timeForGaitPeriod;  // The time of the whole period
        float timePeriod;  // The time of one period
        float timePresent;  
        Vector<float, 4> timeForSwing;   // The swing time for legs, include detach, attach and swing.
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Vector<float, 3> targetCoMVelocity;  // X, Y , alpha in world cordinate
        Vector<float, 3> presentCoMVelocity;  // X, Y , alpha in world cordinate
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        float yawVelocity;   // yaw velocity from imu
        enum _legStatus{stance=0, swing, detach, attach}legStatus[4];  //  0-stance, 1-swing, 2-detach, 3-attach: LF, RF, LH, RH
        Vector<float, 4> timePresentForSwing;
        float L1, L2, L3;  // The length of L
        float width, length;
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> initFootPos;    // initial position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legCmdPos;  // command position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legPresPos;  // present position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legPos_last;
        Matrix<float, 4, 3> legPresVel;  // present velocity of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> jointCmdPos;  // command joint angle 0-11
        Matrix<float, 4, 3>  jointPresPos;  // present motor 0-11
        Matrix<float, 4, 3>  jointPresVel;  // present motor 0-11
        vector<Matrix<float, 3, 3>> jacobian_vector; 
        Matrix<float, 6, 6>A;     //VMC
        Vector<float, 6>B;        //VMC
        Matrix<float, 4, 6>a;     //VMC
        Vector<float, 4>b;        //VMC
        float motorTorque[1];
        float motorInitPos[12];   // init motor angle of quadruped state
        float pid_motortorque[12];
        float jacobian_motortorque[12]; //motor torque in VMC
        float motorCmdTorque[12];
        bool initFlag;

        // the parameters of creeping gait
        float motIniPoCreep[12];    // init motor angle of creeping gait
        float H_onestep;  // The height of one step
        float Yaw_rad;      // The rad of yaw angle
        float k1,k2,k3;
        float L_diag;  // half of body diagonal size
        float beta_diag, alpha_diag; // structural angle of the body
        float v_body_x, v_body_y;    // the velocity of CoM
        float v_leg[4][2];  // the velocity of 4 legs
        float endPosition[4][2];  // the final feet position after one gait cycle
        void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 3> tCV);   
        void setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP);
        void nextStep();
        void oneLegSwing(uint8_t swingLegNum, float length);
        void updateLegStatus();
        void updatejointPresPos(vector<float> jointPos);
        void updatejointPresVel(vector<float> jointVel);
        void updateJacobians();
        void forwardKinematics(int mode);
        void inverseKinematics(Matrix<float, 4, 3> cmdpos);   // standing state
        void updateFtsPresVel(); 
        MotionControl();
};

class IMPControl : public MotionControl
{
    public:
        Matrix<float, 4, 3> target_pos; // LF RF LH RH ; x y z  in CoM cordinate 
        Matrix<float, 4, 3> target_vel;
        Matrix<float, 4, 3> target_acc; // Force in target position
        Matrix<float, 4, 3> target_force;
        // Vector<float, 3> targetCoMVelocity;  // X, Y , alpha in world cordinate
        // Vector<float, 3> presentCoMVelocity;  // X, Y , alpha in world cordinate
        // Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        Matrix<float, 4, 3> xc_dotdot;
        Matrix<float, 4, 3> xc_dot;
        Matrix<float, 4, 3> xc;
        Matrix<float, 3, 4> force, force_last;              // force feedback   x y z ; LF RF LH RH
        Matrix<float, 3, 4> target_torque;
        Matrix<float, 4, 3> K_swing, K_stance, K_detach, K_attach;                     //LF RF LH RH
        Matrix<float, 4, 3> B_swing, B_stance, B_detach, B_attach;
        Matrix<float, 4, 3> M_swing, M_stance, M_detach, M_attach;
        float impCtlRate;

        void updateFtsPresForce(vector<float> torque);
        void updateTargTor(Matrix<float, 3, 4> force);
        void impParaDeliver();
        void impCtller(int mode);
        void impCtllerOneLeg(int legNum);
        void impChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM, int mode);
        IMPControl();
};

void string2float(string add, float* dest);

#endif