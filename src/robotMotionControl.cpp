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
    L1 = 33.5 / 1000;
    L2 = 47.5 / 1000;
    L3 = 23.1 / 1000;
    width = 132.0 / 1000;
    length = 172.0 / 1000;  
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
}

void MotionControl::setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
    timePeriod = tP;
    timeForGaitPeriod = tFGP;
    timeForStancePhase = tFSP;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
}

void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresentPos = initPosition;
    legCmdPos = initPosition;
    targetCoMPosition.setZero();
}
// set  X, Y , alpha in world cordinate
void MotionControl::setCoMVel(Vector<float, 3> tCV)
{
    targetCoMVelocity = tCV;
}

void MotionControl::updateJointPstPos(vector<float> jointPos)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPstPos(i,j) = jointPos[i*3 + j];
    }
}
void MotionControl::     updateJointPstVel(vector<float> jointVel)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPstVel(i,j) = jointVel[i*3 + j];
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

void MotionControl::updateFtsPstPos()
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

void MotionControl::updateFtsPstVel()
{
    for(int i=0; i<4; i++)
    {
        Matrix <float, 3, 1> temp_vel;
        temp_vel = jacobian_vector[i] * jointPstVel.row(i).transpose();
        ftsPstVel.row(i) = temp_vel.transpose();
    }
}

void MotionControl::inverseKinematics()
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
        {
          float factor_y, factor_x, factor_xc, factor_yc, factor_zc;  // factor for x/y; factor for whole formula
          if(legNum==0)
          {
              factor_xc= 1;
              factor_yc= -1;
              factor_zc= -1;
              factor_x= 1;
              factor_y= 1;
          }
          if(legNum==1)
          {
              factor_xc= -1;
              factor_yc= 1;
              factor_zc= 1;
              factor_x= 1;
              factor_y= -1;
          }
          if(legNum==2)
          {
              factor_xc= -1;
              factor_yc= 1;
              factor_zc= -1;
              factor_x= 1;
              factor_y= 1;
          }
          if(legNum==3)
          {
              factor_xc= 1;
              factor_yc= 1;
              factor_zc= 1;
              factor_x= 1;
              factor_y= -1;
          }
          joinCmdPos(legNum,0) = factor_xc * (asin(L3 / sqrt( legCmdPos(legNum,2)*legCmdPos(legNum,2) + legCmdPos(legNum,1)*legCmdPos(legNum,1) )) + atan2(legCmdPos(legNum,2),factor_y * legCmdPos(legNum,1)) );     
          joinCmdPos(legNum,1) = factor_yc * (asin((legCmdPos(legNum,1) * legCmdPos(legNum,1) + legCmdPos(legNum,0) * legCmdPos(legNum,0) + legCmdPos(legNum,2) * legCmdPos(legNum,2) + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (legCmdPos(legNum,1) * legCmdPos(legNum,1) +  legCmdPos(legNum,0) * legCmdPos(legNum,0) + legCmdPos(legNum,2) * legCmdPos(legNum,2) - L3 * L3)))
                  - atan2(sqrt(legCmdPos(legNum,1) * legCmdPos(legNum,1) + legCmdPos(legNum,2) * legCmdPos(legNum,2) - L3 * L3) , factor_x * legCmdPos(legNum,0)));
          joinCmdPos(legNum,2) = factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - legCmdPos(legNum,1) * legCmdPos(legNum,1) - legCmdPos(legNum,0) * legCmdPos(legNum,0) - legCmdPos(legNum,2) * legCmdPos(legNum,2)) / (2 * L1 * L2));
        }
}

void MotionControl::nextStep()
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {
        if(timePresent > timeForStancePhase.row(legNum)(0)-timePeriod/2 && timePresent < timeForStancePhase.row(legNum)(1)+timePeriod/2 )
        {     // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
            if(abs(timePresent - timeForStancePhase.row(legNum)(0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                for(uint8_t pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosition(legNum,2)), -sin(targetCoMPosition(legNum,2)), targetCoMPosition(legNum,0),
                   sin(targetCoMPosition(legNum,2)), cos(targetCoMPosition(legNum,2)), targetCoMPosition(legNum,1),
                   0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<shoulderPos.row(legNum)(0), shoulderPos.row(legNum)(1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(timePresent - timeForStancePhase.row(legNum)(0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForStancePhase.row(legNum)(1)) < 1e-4)  // if on the end pos
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
            stanceFlag(legNum) = true;
        }
        else    //swing phase 
        {
            Matrix<float, 1, 3> swingPhaseVelocity = (stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / 
                                        (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)) - timePeriod);
            
            for(uint8_t pos=0; pos<3; pos++)
                legCmdPos(legNum, pos) = legCmdPos(legNum, pos) - swingPhaseVelocity(pos) * timePeriod;
            
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) > 1e-4)
                legCmdPos(legNum, 2) -= 3.0/1000;
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) < -1e-4 
                && timePresentForSwing(legNum) > 1e-4)
                legCmdPos(legNum, 2) += 3.0/1000;
            stanceFlag(legNum) = false;
        }
    }

    timePresent += timePeriod;
    for(uint8_t leg=0; leg<4; leg++)
    {
        for(uint8_t pos=0; pos<3; pos++)
        {
            targetCoMPosition(leg, pos) += targetCoMVelocity(pos) * timePeriod;
        }
        if(stanceFlag(leg) == 0) timePresentForSwing(leg) += timePeriod;
        else timePresentForSwing(leg) = 0;
    }

    if (abs(timePresent - timeForGaitPeriod - timePeriod) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
    }

}
/////////////////////////////////////////////////////////////////////////////
//                                          IMPControl
//////////////////////////////////////////////////////////////////////////////

IMPControl::IMPControl()
{
    K.setConstant(1000);
    B.setConstant(30);
    M.setConstant(3);
    xc_dotdot.setZero();
    xc_dot.setZero();
    xc.setZero();
    target_pos.setZero();
    target_vel.setZero();
    target_acc.setZero();
    target_force.setZero();
}
//motors.present_torque  ->  force
void IMPControl::impdeliver(vector<float>present_torque)
{
    Matrix<float, 3, 4> temp;
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = present_torque[i*4+j];
    for (int i=0; i<4; i++)
        force.col(i) = jacobian_vector[i].transpose().inverse() * temp.col(i);
    target_pos = legCmdPos;
    //imp.target_ve4l = 0;
    //imp.target_acc = 0; //
    //target_force << 0;
}
void IMPControl::impCtller()
{
    // xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    xc_dotdot =  target_acc +M.cwiseInverse().cwiseProduct( ( target_force - force.transpose() + B.cwiseProduct(target_vel - ftsPstVel) +  K.cwiseProduct(target_pos - ftsPstPos)) ); //
    xc_dot =  ftsPstVel + xc_dotdot * (1/impCtlRate);
    xc =  ftsPstPos + 0.5 * (xc_dot * (1/impCtlRate));
    legCmdPos = xc;
}
