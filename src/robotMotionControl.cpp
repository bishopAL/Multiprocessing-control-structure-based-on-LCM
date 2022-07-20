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
/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void MotionControl::setCoMVel(Vector<float, 3> tCV)
{
    targetCoMVelocity = tCV;
}
/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPstPos(4,3)
 */
void MotionControl::updateJointPstPos(vector<float> jointPos)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPstPos(i,j) = jointPos[i*3 + j];
    }
}
/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPstVel(4,3)
 */
void MotionControl::updateJointPstVel(vector<float> jointVel)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPstVel(i,j) = jointVel[i*3 + j];
    }
}
/**
 * @brief 
 * update jacobian_vector with jointPstPos
 */
void MotionControl::updateJacobians()
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
    {
        float th0,th1,th2;
        float factor_y, factor_x, factor_z, factor_sin;  
        th0 =  jointPstPos(legNum,0);
        th1 =  jointPstPos(legNum,1);
        th2 =  jointPstPos(legNum,2); 
        if(legNum==0 )              //LF  
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_sin = 1;
        }
        else if(legNum==1 )     
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_sin = -1;
        }
        else if(legNum==2 )     
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_sin = -1;
        }
        else if(legNum==3 )     
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_sin = 1;
        }
        jacobian_vector[legNum](0, 0) = 0;
        jacobian_vector[legNum](0, 1) = -factor_x*(L2*sin(th1 + th2) + L1*factor_sin*cos(th1));
        jacobian_vector[legNum](0, 2) = -L2*factor_x*sin(th1 + th2);
        jacobian_vector[legNum](1, 0) = -factor_y*(sin(th0)*(L1*cos(th1) + L2*factor_sin*sin(th1 + th2)) - L3*factor_sin*cos(th0));
        jacobian_vector[legNum](1, 1) = -factor_y*cos(th0)*(L1*sin(th1) - L2*factor_sin*cos(th1 + th2));
        jacobian_vector[legNum](1, 2) = L2*factor_sin*factor_y*cos(th1 + th2)*cos(th0);
        jacobian_vector[legNum](2, 0) = L3*sin(th0) + factor_sin*factor_z*cos(th0)*(L1*cos(th1) + L2*factor_sin*sin(th1 + th2));
        jacobian_vector[legNum](2, 1) = -factor_sin*factor_z*sin(th0)*(L1*sin(th1) - L2*factor_sin*cos(th1 + th2));
        jacobian_vector[legNum](2, 2) = L2*factor_z*cos(th1 + th2)*sin(th0);
    }
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
/**
 * @brief forwardKinematics
 * 
 * @param mode 
 * =0    update legCmdPos with joinCmdPos(target)
 * =1    update ftsPstPos with jointPstPos(present)
 */
void MotionControl::forwardKinematics(int mode)
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
    {
        float th0,th1,th2;
        float factor_y, factor_x,  factor_z, factor_sin;  
        if(mode==0)
        {
            th0 =  joinCmdPos(legNum,0);
            th1 =  joinCmdPos(legNum,1);
            th2 =  joinCmdPos(legNum,2);    
        }
        else if(mode==1)
        {
            th0 =  jointPstPos(legNum,0);
            th1 =  jointPstPos(legNum,1);
            th2 =  jointPstPos(legNum,2);     
        }

        if(legNum==0 )              //LF  
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_sin = 1;
        }
        else if(legNum==1 )          //RF 
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_sin = -1;
        }
        else if(legNum==2 )          //LH    
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_sin = -1;
        }
        else if(legNum==3 )         //RH  
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_sin = 1;
        }
        if(mode==0)
        {
            legCmdPos(legNum,0) = factor_x * (-factor_sin * L1 * sin(th1) + L2 * cos(th1 + th2));
            legCmdPos(legNum,1) = factor_y * ((( L1 * cos(th1) + factor_sin * L2 * sin(th1 + th2) ) * cos(th0) + factor_sin * L3 * sin(th0)));
            legCmdPos(legNum,2) = factor_z * (( L1 * cos(th1) + factor_sin * L2 * sin(th1 + th2) ) * factor_sin * sin(th0) -  L3 * cos(th0));
        }
        else if(mode==1)
        {
            ftsPstPos(legNum,0) = factor_x * (-factor_sin * L1 * sin(th1) + L2 * cos(th1 + th2));
            ftsPstPos(legNum,1) = factor_y * ((( L1 * cos(th1) + factor_sin * L2 * sin(th1 + th2) ) * cos(th0) + factor_sin * L3 * sin(th0)));
            ftsPstPos(legNum,2) = factor_z * (( L1 * cos(th1) + factor_sin * L2 * sin(th1 + th2) ) * factor_sin * sin(th0) -  L3 * cos(th0));
        }
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
            else if(legNum==1)
            {
                factor_xc= -1;
                factor_yc= 1;
                factor_zc= 1;
                factor_x= 1;
                factor_y= -1;
            }
            else if(legNum==2)
            {
                factor_xc= -1;
                factor_yc= 1;
                factor_zc= -1;
                factor_x= 1;
                factor_y= 1;
            }
            else if(legNum==3)
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
            // for(int k=0;k<3;k++)
            //     if( isnanf(joinCmdPos(legNum,k)) )
            //         joinCmdPos(legNum,k) = 0;

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
                legCmdPos(legNum, 2) -= 2.0/1000;
            if( ( timePresentForSwing(legNum) - (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)))/2 ) < -1e-4 
                && timePresentForSwing(legNum) > 1e-4)
                legCmdPos(legNum, 2) += 2.0/1000;
            stanceFlag(legNum) = false;
        }
    }

    timePresent += timePeriod;
    for(uint8_t leg=0; leg<4; leg++)
    {
        for(uint8_t pos=0; pos<3; pos++)
        {
            targetCoMPosition(leg, pos) += targetCoMVelocity(pos) * timePeriod / (timeForStancePhase(leg,1) - timeForStancePhase(leg,0));
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
    float impdata[100];
    string2float("../include/imp_parameter.csv",impdata);
    Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata), mapB(impdata +12), mapM(impdata +24);
    K=mapK;
    B=mapB;
    M=mapM;
    cout<<impdata<<endl;

    xc_dotdot.setZero();
    xc_dot.setZero();
    xc.setZero();
    target_pos.setZero();
    target_vel.setZero();
    target_acc.setZero();
    target_force.setZero();
    impCtlRate = 100;
}

/**
 * @brief 
 * Deliver parameter to impCtller.
 * (target_pos = legCmdPos)
 * @param present_torque update force with present_torque
 */
void IMPControl::impdeliver(vector<float> present_torque)
{
    Matrix<float, 3, 4> temp;
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = present_torque[i*4+j];
    for (int i=0; i<4; i++)
        force.col(i) = jacobian_vector[i].transpose().inverse() * temp.col(i);
    //target_pos = legCmdPos;
    
    // target_vel << 
    // 0.01, 0.01, 0.01, 0.01,
    // 0.01, 0.01, 0.01, 0.01,
    // 0.01, 0.01, 0.01, 0.01;
    //target_acc = 0; //
    //target_force << 0;
}
/**
 * @brief 
 * Calculcate legCmdPos with target_acc, target_force, target_vel, target_pos.
 */
void IMPControl::impCtller()
{
    Matrix<float, 4,3> matrix_temp;
    // xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    xc_dotdot =  target_acc +M.cwiseInverse().cwiseProduct( ( target_force - force.transpose() + B.cwiseProduct(target_vel - ftsPstVel) +  K.cwiseProduct(target_pos - ftsPstPos)) ); //
    xc_dot =  ftsPstVel + xc_dotdot * (1/impCtlRate);
    xc =  ftsPstPos + 0.5 * (xc_dot * (1/impCtlRate));
    legCmdPos = xc;
}

/**
 * @brief 
 * Open the file to read float data to dest.    
 * In the file, ',' must be used after every data and must be the last character.  
 * @param add The address of the file to read, like "../include/init_Motor_angle.csv"
 * @param dest Floating pointer to store datas.
 */
void string2float(string add, float* dest)
{
    char data_char[8000],*char_float;
    const char *a=",";  //Separate datas
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
        //cout<<'|'<<dest[i-1]<<endl;
        char_float=strtok(NULL, a);
    }
    inidata.close();
}