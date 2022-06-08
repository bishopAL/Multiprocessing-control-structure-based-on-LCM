#include "robotMotionControl.h"

MotionControl::MotionControl()
{

initFlag = false;
timePeriod = tP;
timePresent = 0.0;
timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
targetCoMVelocity << 0.0, 0.0, 0.0;
L1 = 33.5;
L2 = 47.5;
L3 = 23.1;
width = 132.0;
length = 172.0;  
shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
}

void MotionControl::updateJointPstPos(vector<float> jointPos)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            
        }
    }
}

void MotionControl::updateJacobians()
{
    jacobian_vector(0, 0, 0) = 0;
    // [                                                                                                                            0,                                                                   - L2*sin(theta2 + theta3) - L1*cos(theta2),                                                        -L2*sin(theta2 + theta3)]
    // [L3*cos(theta1) - L1*cos(theta2)*sin(theta1) - L2*cos(theta2)*sin(theta1)*sin(theta3) - L2*cos(theta3)*sin(theta1)*sin(theta2), L2*cos(theta1)*cos(theta2)*cos(theta3) - L1*cos(theta1)*sin(theta2) - L2*cos(theta1)*sin(theta2)*sin(theta3), L2*cos(theta1)*cos(theta2)*cos(theta3) - L2*cos(theta1)*sin(theta2)*sin(theta3)]
    // [L3*sin(theta1) + L1*cos(theta1)*cos(theta2) + L2*cos(theta1)*cos(theta2)*sin(theta3) + L2*cos(theta1)*cos(theta3)*sin(theta2), L2*cos(theta2)*cos(theta3)*sin(theta1) - L1*sin(theta1)*sin(theta2) - L2*sin(theta1)*sin(theta2)*sin(theta3), L2*cos(theta2)*cos(theta3)*sin(theta1) - L2*sin(theta1)*sin(theta2)*sin(theta3)]
 
}