#include "robotMotionControl.h"

MotionControl::MotionControl()
{
Matrix<float, 3, 3> temp;
temp << 0, 0, 0, 0, 0, 0, 0, 0, 0;
for(uint8_t i=0; i<4; i++)
jacobian_vector.push_back(temp);

initFlag = false;
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
            jointPstPos(i,j) = jointPos[i*3 + j];
        }
    }
}

void MotionControl::updateJacobians()
{
    // LF
    jacobian_vector[0](0, 0) = 0;
    jacobian_vector[0](0, 1) = - L2*sin(jointPstPos(0,1) + jointPstPos(0,2)) - L1*cos(jointPstPos(0,1));
    jacobian_vector[0](0, 2) = -L2*sin(jointPstPos(0,1) + jointPstPos(0,2));
    jacobian_vector[0](1, 0) = L3*cos(jointPstPos(0,0)) - L1*cos(jointPstPos(0,1))*sin(jointPstPos(0,0)) - L2*cos(jointPstPos(0,1))*sin(jointPstPos(0,0))*sin(jointPstPos(0,2)) - L2*cos(jointPstPos(0,2))*sin(jointPstPos(0,0))*sin(jointPstPos(0,1));
    jacobian_vector[0](1, 1) = L2*cos(jointPstPos(0,0))*cos(jointPstPos(0,1))*cos(jointPstPos(0,2)) - L1*cos(jointPstPos(0,0))*sin(jointPstPos(0,1)) - L2*cos(jointPstPos(0,0))*sin(jointPstPos(0,1))*sin(jointPstPos(0,2));
    jacobian_vector[0](1, 2) = L2*cos(jointPstPos(0,0))*cos(jointPstPos(0,1))*cos(jointPstPos(0,2)) - L2*cos(jointPstPos(0,0))*sin(jointPstPos(0,1))*sin(jointPstPos(0,2));
    jacobian_vector[0](2, 0) = L3*sin(jointPstPos(0,0)) + L1*cos(jointPstPos(0,0))*cos(jointPstPos(0,1)) + L2*cos(jointPstPos(0,0))*cos(jointPstPos(0,1))*sin(jointPstPos(0,2)) + L2*cos(jointPstPos(0,0))*cos(jointPstPos(0,2))*sin(jointPstPos(0,1));
    jacobian_vector[0](2, 1) = L2*cos(jointPstPos(0,1))*cos(jointPstPos(0,2))*sin(jointPstPos(0,0)) - L1*sin(jointPstPos(0,0))*sin(jointPstPos(0,1)) - L2*sin(jointPstPos(0,0))*sin(jointPstPos(0,1))*sin(jointPstPos(0,2));
    jacobian_vector[0](2, 2) = L2*cos(jointPstPos(0,1))*cos(jointPstPos(0,2))*sin(jointPstPos(0,0)) - L2*sin(jointPstPos(0,0))*sin(jointPstPos(0,1))*sin(jointPstPos(0,2));

    // RF
    jacobian_vector[1](0, 0) = 0;
    jacobian_vector[1](0, 1) = L1*cos(jointPstPos(1,1)) - L2*sin(jointPstPos(1,1) + jointPstPos(1,2));
    jacobian_vector[1](0, 2) = -L2*sin(jointPstPos(1,1) + jointPstPos(1,2));
    jacobian_vector[1](1, 0) = L3*cos(jointPstPos(1,0)) + L1*cos(jointPstPos(1,1))*sin(jointPstPos(1,0)) - L2*cos(jointPstPos(1,1))*sin(jointPstPos(1,0))*sin(jointPstPos(1,2)) - L2*cos(jointPstPos(1,2))*sin(jointPstPos(1,0))*sin(jointPstPos(1,1));
    jacobian_vector[1](1, 1) = L1*cos(jointPstPos(1,0))*sin(jointPstPos(1,1)) + L2*cos(jointPstPos(1,0))*cos(jointPstPos(1,1))*cos(jointPstPos(1,2)) - L2*cos(jointPstPos(1,0))*sin(jointPstPos(1,1))*sin(jointPstPos(1,2));
    jacobian_vector[1](1, 2) = L2*cos(jointPstPos(1,0))*cos(jointPstPos(1,1))*cos(jointPstPos(1,2)) - L2*cos(jointPstPos(1,0))*sin(jointPstPos(1,1))*sin(jointPstPos(1,2));
    jacobian_vector[1](2, 0) = L3*sin(jointPstPos(1,0)) - L1*cos(jointPstPos(1,0))*cos(jointPstPos(1,1)) + L2*cos(jointPstPos(1,0))*cos(jointPstPos(1,1))*sin(jointPstPos(1,2)) + L2*cos(jointPstPos(1,0))*cos(jointPstPos(1,2))*sin(jointPstPos(1,1));
    jacobian_vector[1](2, 1) = L1*sin(jointPstPos(1,0))*sin(jointPstPos(1,1)) + L2*cos(jointPstPos(1,1))*cos(jointPstPos(1,2))*sin(jointPstPos(1,0)) - L2*sin(jointPstPos(1,0))*sin(jointPstPos(1,1))*sin(jointPstPos(1,2));
    jacobian_vector[1](2, 2) = L2*cos(jointPstPos(1,1))*cos(jointPstPos(1,2))*sin(jointPstPos(1,0)) - L2*sin(jointPstPos(1,0))*sin(jointPstPos(1,1))*sin(jointPstPos(1,2));

    //LH
    jacobian_vector[2](0, 0) = 0;
    jacobian_vector[2](0, 1) = L2*sin(jointPstPos(2,1) + jointPstPos(2,2)) - L1*cos(jointPstPos(2,1));
    jacobian_vector[2](0, 2) = L2*sin(jointPstPos(2,1) + jointPstPos(2,2));
    jacobian_vector[2](1, 0) = L2*cos(jointPstPos(2,1))*sin(jointPstPos(2,0))*sin(jointPstPos(2,2)) - L1*cos(jointPstPos(2,1))*sin(jointPstPos(2,0)) - L3*cos(jointPstPos(2,0)) + L2*cos(jointPstPos(2,2))*sin(jointPstPos(2,0))*sin(jointPstPos(2,1));
    jacobian_vector[2](1, 1) = L2*cos(jointPstPos(2,0))*sin(jointPstPos(2,1))*sin(jointPstPos(2,2)) - L2*cos(jointPstPos(2,0))*cos(jointPstPos(2,1))*cos(jointPstPos(2,2)) - L1*cos(jointPstPos(2,0))*sin(jointPstPos(2,1));
    jacobian_vector[2](1, 2) = L2*cos(jointPstPos(2,0))*sin(jointPstPos(2,1))*sin(jointPstPos(2,2)) - L2*cos(jointPstPos(2,0))*cos(jointPstPos(2,1))*cos(jointPstPos(2,2));
    jacobian_vector[2](2, 0) = L3*sin(jointPstPos(2,0)) - L1*cos(jointPstPos(2,0))*cos(jointPstPos(2,1)) + L2*cos(jointPstPos(2,0))*cos(jointPstPos(2,1))*sin(jointPstPos(2,2)) + L2*cos(jointPstPos(2,0))*cos(jointPstPos(2,2))*sin(jointPstPos(2,1));
    jacobian_vector[2](2, 1) = L1*sin(jointPstPos(2,0))*sin(jointPstPos(2,1)) + L2*cos(jointPstPos(2,1))*cos(jointPstPos(2,2))*sin(jointPstPos(2,0)) - L2*sin(jointPstPos(2,0))*sin(jointPstPos(2,1))*sin(jointPstPos(2,2));
    jacobian_vector[2](2, 2) = L2*cos(jointPstPos(2,1))*cos(jointPstPos(2,2))*sin(jointPstPos(2,0)) - L2*sin(jointPstPos(2,0))*sin(jointPstPos(2,1))*sin(jointPstPos(2,2));

    //RH
    jacobian_vector[3](0, 0) = 0;
    jacobian_vector[3](0, 1) = L2*sin(jointPstPos(3,1) + jointPstPos(3,2)) + L1*cos(jointPstPos(3,1));
    jacobian_vector[3](0, 2) = L2*sin(jointPstPos(3,1) + jointPstPos(3,2));
    jacobian_vector[3](1, 0) = L3*cos(jointPstPos(3,0)) + L1*cos(jointPstPos(3,1))*sin(jointPstPos(3,0)) + L2*cos(jointPstPos(3,1))*sin(jointPstPos(3,0))*sin(jointPstPos(3,2)) + L2*cos(jointPstPos(3,2))*sin(jointPstPos(3,0))*sin(jointPstPos(3,1));
    jacobian_vector[3](1, 1) = L1*cos(jointPstPos(3,0))*sin(jointPstPos(3,1)) - L2*cos(jointPstPos(3,0))*cos(jointPstPos(3,1))*cos(jointPstPos(3,2)) + L2*cos(jointPstPos(3,0))*sin(jointPstPos(3,1))*sin(jointPstPos(3,2));
    jacobian_vector[3](1, 2) = L2*cos(jointPstPos(3,0))*sin(jointPstPos(3,1))*sin(jointPstPos(3,2)) - L2*cos(jointPstPos(3,0))*cos(jointPstPos(3,1))*cos(jointPstPos(3,2));
    jacobian_vector[3](2, 0) = L3*sin(jointPstPos(3,0)) - L1*cos(jointPstPos(3,0))*cos(jointPstPos(3,1)) - L2*cos(jointPstPos(3,0))*cos(jointPstPos(3,1))*sin(jointPstPos(3,2)) - L2*cos(jointPstPos(3,0))*cos(jointPstPos(3,2))*sin(jointPstPos(3,1));
    jacobian_vector[3](2, 1) = L1*sin(jointPstPos(3,0))*sin(jointPstPos(3,1)) - L2*cos(jointPstPos(3,1))*cos(jointPstPos(3,2))*sin(jointPstPos(3,0)) + L2*sin(jointPstPos(3,0))*sin(jointPstPos(3,1))*sin(jointPstPos(3,2));
    jacobian_vector[3](2, 2) = L2*sin(jointPstPos(3,0))*sin(jointPstPos(3,1))*sin(jointPstPos(3,2)) - L2*cos(jointPstPos(3,1))*cos(jointPstPos(3,2))*sin(jointPstPos(3,0));
}

void MotionControl::forwardKinematics()
{
    ftsPstPos(0, 0) = L2 * cos(jointPstPos(0,1) + jointPstPos(0,2)) - L1 * sin(jointPstPos(0,1));
    ftsPstPos(0, 1) = L3 * sin(jointPstPos(0,0)) + L1 * cos(jointPstPos(0,0)) * cos(jointPstPos(0,1)) + L2 * cos(jointPstPos(0,0)) * cos(jointPstPos(0,1)) * sin(jointPstPos(0,2)) + L2 * cos(jointPstPos(0,0)) * cos(jointPstPos(0,2)) * sin(jointPstPos(0,1));
    ftsPstPos(0, 2) =  - L3 * cos(jointPstPos(0,0)) + L1 * sin(jointPstPos(0,0)) * cos(jointPstPos(0,1)) + L2 * sin(jointPstPos(0,0)) * cos(jointPstPos(0,1)) * sin(jointPstPos(0,2)) + L2 * sin(jointPstPos(0,0)) * cos(jointPstPos(0,2)) * sin(jointPstPos(0,1));
    ftsPstPos(1, 0) = L2 * cos(jointPstPos(1, 1) + jointPstPos(1, 2)) + L1 * sin(jointPstPos(1, 1));
    ftsPstPos(1, 1) = L3 * sin(jointPstPos(1, 0)) - L1 * cos(jointPstPos(1, 0)) * cos(jointPstPos(1, 1)) + L2 * cos(jointPstPos(1, 0)) * cos(jointPstPos(1, 1)) * sin(jointPstPos(1, 2)) + L2 * cos(jointPstPos(1, 0)) * cos(jointPstPos(1, 2)) * sin(jointPstPos(1, 1));
    ftsPstPos(1, 2) = - L3 * cos(jointPstPos(1, 0)) - L1 * sin(jointPstPos(1, 0)) * cos(jointPstPos(1, 1)) + L2 * sin(jointPstPos(1, 0)) * cos(jointPstPos(1, 1)) * sin(jointPstPos(1, 2)) + L2 * sin(jointPstPos(1, 0)) * cos(jointPstPos(1, 2)) * sin(jointPstPos(1, 1));
    ftsPstPos(2, 0) = - L2 * cos(jointPstPos(2, 1) + jointPstPos(2, 2)) - L1 * sin(jointPstPos(2, 1));
    ftsPstPos(2, 1) = - L3 * sin(jointPstPos(2, 0)) + L1 * cos(jointPstPos(2, 0)) * cos(jointPstPos(2, 1)) - L2 * cos(jointPstPos(2, 0)) * cos(jointPstPos(2, 1)) * sin(jointPstPos(2, 2)) - L2 * cos(jointPstPos(2, 0)) * cos(jointPstPos(2, 2)) * sin(jointPstPos(2, 1));
    ftsPstPos(2, 2) = - L3 * cos(jointPstPos(2, 0)) - L1 * sin(jointPstPos(2, 0)) * cos(jointPstPos(2, 1)) + L2 * sin(jointPstPos(2, 0)) * cos(jointPstPos(2, 1)) * sin(jointPstPos(2, 2)) + L2 * sin(jointPstPos(2, 0)) * cos(jointPstPos(2, 2)) * sin(jointPstPos(2, 1));
    ftsPstPos(3, 0) = - L2 * cos(jointPstPos(3, 1) + jointPstPos(3, 2)) + L1 * sin(jointPstPos(3, 1));
    ftsPstPos(3, 1) = L3 * sin(jointPstPos(3, 0)) - L1 * cos(jointPstPos(3, 0)) * cos(jointPstPos(3, 1)) - L2 * cos(jointPstPos(3, 0)) * cos(jointPstPos(3, 1)) * sin(jointPstPos(3, 2)) - L2 * cos(jointPstPos(3, 0)) * cos(jointPstPos(3, 2)) * sin(jointPstPos(3, 1));
    ftsPstPos(3, 2) = - L3 * cos(jointPstPos(3, 0)) - L1 * sin(jointPstPos(3, 0)) * cos(jointPstPos(3, 1)) - L2 * sin(jointPstPos(3, 0)) * cos(jointPstPos(3, 1)) * sin(jointPstPos(3, 2)) - L2 * sin(jointPstPos(3, 0)) * cos(jointPstPos(3, 2)) * sin(jointPstPos(3, 1));
}

void MotionControl::inverseKinematics()
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
        {
          float factor_y, factor_x, factor_xc, factor_yc, factor_zc;  // factor for x/y; factor for whole formula
          if(legNum==0)
          {
              factor_xc=-1;
              factor_yc=1;
              factor_zc=1;
              factor_x=1;
              factor_y=1;
          }
          if(legNum==1)
          {
              factor_xc=1;
              factor_yc=-1;
              factor_zc=-1;
              factor_x=1;
              factor_y=-1;
          }
          if(legNum==2)
          {
              factor_xc=-1;
              factor_yc=-1;
              factor_zc=-1;
              factor_x=-1;
              factor_y=1;
          }
          if(legNum==3)
          {
              factor_xc=1;
              factor_yc=1;
              factor_zc=1;
              factor_x=-1;
              factor_y=-1;
          }
          cmdJointPos(legNum,1) = -factor_xc * (asin(L3 / sqrt( cmdFootPos(legNum,2)*cmdFootPos(legNum,2) + cmdFootPos(legNum,1)*cmdFootPos(legNum,1) )) + atan2(cmdFootPos(legNum,2),factor_y * cmdFootPos(legNum,1)) );     
          cmdJointPos(legNum,0) = -factor_yc * (asin((cmdFootPos(legNum,1) * cmdFootPos(legNum,1) + cmdFootPos(legNum,0) * cmdFootPos(legNum,0) + cmdFootPos(legNum,2) * cmdFootPos(legNum,2) + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (cmdFootPos(legNum,1) * cmdFootPos(legNum,1) +  cmdFootPos(legNum,0) * cmdFootPos(legNum,0) + cmdFootPos(legNum,2) * cmdFootPos(legNum,2) - L3 * L3)))
                  - atan2(sqrt(cmdFootPos(legNum,1) * cmdFootPos(legNum,1) + cmdFootPos(legNum,2) * cmdFootPos(legNum,2) - L3 * L3) , factor_x * cmdFootPos(legNum,0)));
          cmdJointPos(legNum,2) = -factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - cmdFootPos(legNum,1) * cmdFootPos(legNum,1) - cmdFootPos(legNum,0) * cmdFootPos(legNum,0) - cmdFootPos(legNum,2) * cmdFootPos(legNum,2)) / (2 * L1 * L2));
        }
}