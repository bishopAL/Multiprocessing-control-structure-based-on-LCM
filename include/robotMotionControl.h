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
        float timeOneSwingPeriod;  // The swing time for diag legs
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Vector<float, 3> targetCoMVelocity;  // X, Y , alpha in world cordinate
        Vector<float, 3> presentCoMVelocity;  // X, Y , alpha in world cordinate
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        float yawVelocity;   // yaw velocity from imu
        Vector<bool, 4> stanceFlag;  // True, False: LF, RF, LH, RH
        Vector<float, 4> timePresentForSwing;
        float L1, L2, L3;  // The length of L
        float width, length;
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in shoulder coordinate
        Matrix<float, 4, 3> legCmdPos;  // command X-Y-Z: LF, RF, LH, RH in shoulder coordinate
        Matrix<float, 4, 3> leg2CoMPrePos;  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 3> leg2CoMCmdPos;  // command X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 3> ftsPstPos;  // Pos of foot  in shoulder coordinate
        Matrix<float, 4, 3> ftsPstVel;  // Vel of foot  in shoulder coordinate
        Matrix<float, 4, 3> joinCmdPos;  // command joint angle 0-11
        Matrix<float, 4, 3>  jointPstPos;  // present motor 0-11
        Matrix<float, 4, 3>  jointPstVel;  // present motor 0-11
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
        void updateJointPstPos(vector<float> jointPos);
        void updateJointPstVel(vector<float> jointVel);
        void updateJacobians();
        void inverseKinematics();   // standing state
        void updateFtsPstPos(); // to update Pos of foot  in shoulder coordinate
        void updateFtsPstVel(); // to update Vel of foot  in shoulder coordinate
        MotionControl();
};

class IMPControl : public MotionControl
{
    public:
        Matrix<float, 4, 3> target_pos; // LF RH LH RH ; x y z  in CoM cordinate 
        Matrix<float, 4, 3> target_vel;
        Matrix<float, 4, 3> target_acc; // Force in target position
        // Vector<float, 3> targetCoMVelocity;  // X, Y , alpha in world cordinate
        // Vector<float, 3> presentCoMVelocity;  // X, Y , alpha in world cordinate
        // Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        Matrix<float, 4, 3> xc_dotdot;
        Matrix<float, 4, 3> xc_dot;
        Matrix<float, 4, 3> xc;
        Matrix<float, 3, 4> force;              // force feedback   x y z ; LF RH LH RH
        Matrix<float, 4, 3> K;                     //LF RH LH RH
        Matrix<float, 4, 3> B;
        Matrix<float, 4, 3> M;

        void impdeliver(vector<float>present_torque);
        void impCtller();
        IMPControl();
};