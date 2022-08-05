#include "robotMotionControl.h"

#define StepHeight  55.0/1000
#define ForceLPF  0.9

/**
 * @brief 
 * Open the file to read float data to dest.    
 * In the file, ',' must be used after every data, including the last data.  
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
    inidata.read(data_char,8000);
    char_float=strtok(data_char, a);
    while(char_float!=NULL)
    {        
        dest[i++] = stof(char_float);
        //cout<<'|'<<dest[i-1]<<endl;
        char_float=strtok(NULL, a);
    }
    inidata.close();
}

MotionControl::MotionControl()
{
    Matrix<float, 3, 3> temp;
    temp << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    for(uint8_t i=0; i<4; i++)
        jacobian_vector.push_back(temp);

    initFlag = false;
    stepFlag << 0,0,0,0;
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
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(timeForStancePhase(legNum,0) < timeForStancePhase(legNum,1))
            timeForSwing(legNum) = (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0) - timePeriod));
        else
            timeForSwing(legNum) = timeForStancePhase(legNum,0) - timeForStancePhase(legNum,1) - timePeriod;
    }
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
        float factor_y, factor_x, factor_z, factor_s12, factor_s0;  
        th0 =  jointPstPos(legNum,0);
        th1 =  jointPstPos(legNum,1);
        th2 =  jointPstPos(legNum,2); 
        //  shoulder coordinate to COM
        if(legNum==0 )              //LF  
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_s12 = 1;
            factor_s0 = 1;
        }
        else if(legNum==1 )          //RF 
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_s12 = -1;
            factor_s0 = -1;
        }
        else if(legNum==2 )          //LH    
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_s12 = 1;
            factor_s0 = -1;
        }
        else if(legNum==3 )         //RH  
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_s12 = -1;
            factor_s0 = 1;
        }
        jacobian_vector[legNum](0, 0) = 0;
        jacobian_vector[legNum](0, 1) = -factor_x*(L2*sin(th1 + th2) + L1*factor_s12*cos(th1));
        jacobian_vector[legNum](0, 2) = -L2*factor_x*sin(th1 + th2);
        jacobian_vector[legNum](1, 0) = -factor_y*(sin(th0)*(L1*cos(th1) + L2*factor_s12*sin(th1 + th2)) - L3*factor_s0*cos(th0));
        jacobian_vector[legNum](1, 1) = -factor_y*cos(th0)*(L1*sin(th1) - L2*factor_s12*cos(th1 + th2));
        jacobian_vector[legNum](1, 2) = L2*factor_s12*factor_s0*factor_y*cos(th1 + th2)*cos(th0);
        jacobian_vector[legNum](2, 0) = L3*sin(th0) + factor_s0*factor_z*cos(th0)*(L1*cos(th1) + L2*factor_s12*sin(th1 + th2));
        jacobian_vector[legNum](2, 1) = -factor_s0*factor_z*sin(th0)*(L1*sin(th1) - L2*factor_s12*cos(th1 + th2));
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
        float factor_y, factor_x,  factor_z, factor_s12, factor_s0;  
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
        //  shoulder coordinate to COM
        if(legNum==0 )              //LF  
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_s12 = 1;
            factor_s0 = 1;
        }
        else if(legNum==1 )          //RF 
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_s12 = -1;
            factor_s0 = -1;
        }
        else if(legNum==2 )          //LH    
        {
            factor_x = 1;
            factor_y = 1;
            factor_z = 1;
            factor_s12 = 1;
            factor_s0 = -1;
        }
        else if(legNum==3 )         //RH  
        {
            factor_x = 1;
            factor_y = -1;
            factor_z = 1;
            factor_s12 = -1;
            factor_s0 = 1;
        }
        if(mode==0)
        {
            legCmdPos(legNum,0) = factor_x * (-factor_s12 * L1 * sin(th1) + L2 * cos(th1 + th2));
            legCmdPos(legNum,1) = factor_y * ((( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * cos(th0) + factor_s0 * L3 * sin(th0)));
            legCmdPos(legNum,2) = factor_z * (( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * factor_s0 * sin(th0) -  L3 * cos(th0));
        }
        else if(mode==1)
        {
            ftsPstPos(legNum,0) = factor_x * (-factor_s12 * L1 * sin(th1) + L2 * cos(th1 + th2));
            ftsPstPos(legNum,1) = factor_y * ((( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * cos(th0) + factor_s0 * L3 * sin(th0)));
            ftsPstPos(legNum,2) = factor_z * (( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * factor_s0 * sin(th0) -  L3 * cos(th0));
        }
    }

}

void MotionControl::inverseKinematics(Matrix<float, 4, 3> cmdpos)
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
                factor_yc= -1;
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
            joinCmdPos(legNum,0) = factor_xc * (asin(L3 / sqrt( cmdpos(legNum,2)*cmdpos(legNum,2) + cmdpos(legNum,1)*cmdpos(legNum,1) )) + atan2(cmdpos(legNum,2),factor_y * cmdpos(legNum,1)) );     
            joinCmdPos(legNum,1) = factor_yc * (asin((cmdpos(legNum,1) * cmdpos(legNum,1) + cmdpos(legNum,0) * cmdpos(legNum,0) + cmdpos(legNum,2) * cmdpos(legNum,2) + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (cmdpos(legNum,1) * cmdpos(legNum,1) +  cmdpos(legNum,0) * cmdpos(legNum,0) + cmdpos(legNum,2) * cmdpos(legNum,2) - L3 * L3)))
                    - atan2(sqrt(cmdpos(legNum,1) * cmdpos(legNum,1) + cmdpos(legNum,2) * cmdpos(legNum,2) - L3 * L3) , factor_x * cmdpos(legNum,0)));
            joinCmdPos(legNum,2) = factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - cmdpos(legNum,1) * cmdpos(legNum,1) - cmdpos(legNum,0) * cmdpos(legNum,0) - cmdpos(legNum,2) * cmdpos(legNum,2)) / (2 * L1 * L2));

        }
}

void MotionControl::nextStep()
{

    if (abs(timePresent - timeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(timeForStancePhase(legNum,0) < timeForStancePhase(legNum,1))
        {
            if(timePresent > timeForStancePhase(legNum,0) - timePeriod/2 && timePresent < timeForStancePhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                stepFlag(legNum) = 0;
            else    //swing phase 
                stepFlag(legNum) = 1;            
        }
        else
        {
            if(timePresent > timeForStancePhase(legNum,0) - timePeriod/2 || timePresent < timeForStancePhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                stepFlag(legNum) = 0;
            else    //swing phase 
                stepFlag(legNum) = 1;
        }
        if(stepFlag(legNum) == 0 ) //stance phase
        {     
            if(abs(timePresent - timeForStancePhase(legNum,0)) < 1e-4)  // if on the start pos 
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
            oneShoulderPos_3x1<<shoulderPos(legNum,0), shoulderPos(legNum,1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(timePresent - timeForStancePhase(legNum,0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForStancePhase(legNum,1)) < timePeriod + 1e-4)  // if on the end pos   
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
        }
        else    //swing phase 
        {
            Matrix<float, 1, 3> swingPhaseVelocity = (stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / (timeForSwing(legNum) - timePeriod);
            //cout<<"legNum_"<<(int)legNum<<":"<<swingPhaseVelocity.array()<<"  ";
            for(uint8_t pos=0; pos<3; pos++)
                legCmdPos(legNum, pos) = legCmdPos(legNum, pos) - swingPhaseVelocity(pos) * timePeriod;

            if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) < -1e-4 
                && timePresentForSwing(legNum) > 1e-4)
                legCmdPos(legNum, 2) += StepHeight / timeForSwing(legNum) * timePeriod;
            if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) > 1e-4)
                legCmdPos(legNum, 2) -=  StepHeight / timeForSwing(legNum) * timePeriod;
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag(legNum)<<"  ";
    }

    
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        for(uint8_t pos=0; pos<3; pos++)
        {
            targetCoMPosition(legNum, pos) += targetCoMVelocity(pos) * timePeriod / timeForSwing(legNum);
        }
        if(stepFlag(legNum) != 0) timePresentForSwing(legNum) += timePeriod;
        else timePresentForSwing(legNum) = 0;   //stance phase
    }
    timePresent += timePeriod;
}
/////////////////////////////////////////////////////////////////////////////
//                                          IMPControl
//////////////////////////////////////////////////////////////////////////////

IMPControl::IMPControl()
{
    float impdata[200];
    string2float("../include/imp_parameter.csv",impdata);   //0-stance, 1-swing, 2-detach, 3-attach
    //Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata), mapB(impdata + 12 * 1), mapM(impdata + 12 * 2);
    for(int i=0; i<4; i++)
    {
        Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata + 36 * i), mapB(impdata + 12 + 36 * i), mapM(impdata  + 24 + 36 * i);
        impChangePara(mapK,mapB,mapM, i);
    } 

    xc_dotdot.setZero();
    xc_dot.setZero();
    xc.setZero();
    target_pos.setZero();
    target_vel.setZero();
    target_acc.setZero();
    target_force.setZero();
    force_last.setZero();
    impCtlRate = 100;
}

/**
 * @brief 
 * Calcute feedback force for impCtller.
 * @param torque update force with present_torque
 */
void IMPControl::impFeedback(vector<float> torque)
{
    Matrix<float, 3, 4> temp;
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = torque[i+j*3];
    for (int i=0; i<4; i++)
        force.col(i) = ForceLPF * force_last.col(i) - (1-ForceLPF) * jacobian_vector[i].transpose().inverse() * temp.col(i);
    force_last = force;
}

/**
 * @brief 
 * Deliver parameter to impCtller, after run nextstep().
 */
void IMPControl::impParaDeliver()
{
    target_pos = legCmdPos;
    // target_force << 
    // 0, 0, 1.0,
    // 0, 0, 1.0,
    // 0, 0, 1.0,
    // 0, 0, 1.0;
    for(uint8_t legNum=0; legNum<4; legNum++)
    {   
        if(stepFlag(legNum) == 1) //swing
        {
            if( ( timePresentForSwing(legNum) - (timeForSwing(legNum) - timePeriod *8) ) > -1e-4 )
            {
                stepFlag(legNum) = 3;   //attach
                target_force.row(legNum) << 0, 0, 1.0;  // x, y, z
            }
            else if( ( timePresentForSwing(legNum) - timePeriod *8 ) < 1e-4 && timePresentForSwing(legNum) > 1e-4)
            {
                stepFlag(legNum) = 2;   //detach
                target_force.row(legNum) << 0, 0, -1.0;
            }
            else    //swing
            {
                stepFlag(legNum) = 1;
                target_force.row(legNum) << 0, 0, 0;
            }
        }
        else        //stance
        {
            stepFlag(legNum) = 0;
            target_force << 0, 0, 0,
                            0, 0, 0,
                            0, 0, 0,
                            0, 0, 0;
        }
    }

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
    for(uint8_t legNum=0; legNum<4; legNum++)
    {   
        switch(stepFlag(legNum))
        {
            case 0: //stance
            {
                xc_dotdot.row(legNum) =  target_acc.row(legNum) - M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )
                + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - ftsPstVel.row(legNum)) 
                + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - ftsPstPos.row(legNum)); 
                break;
            }
            case 1: //swing
            {
                xc_dotdot.row(legNum) =  target_acc.row(legNum) - M_swing.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )
                + B_swing.row(legNum).cwiseProduct(target_vel.row(legNum) - ftsPstVel.row(legNum)) 
                + K_swing.row(legNum).cwiseProduct(target_pos.row(legNum) - ftsPstPos.row(legNum)); 
                break;
            }
            case 2: //detach
            {
                xc_dotdot.row(legNum) =  target_acc.row(legNum) - M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )
                + B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - ftsPstVel.row(legNum)) 
                + K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - ftsPstPos.row(legNum));
                break;
            }
            case 3: //attach
            {
                xc_dotdot.row(legNum) =  target_acc.row(legNum) - M_attach.row(legNum).cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )
                + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - ftsPstVel.row(legNum)) 
                + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - ftsPstPos.row(legNum));
                cout<<"M__attach_"<<(int)legNum<<"  "<<-M_attach.row(legNum).cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                cout<<"B__attach_"<<(int)legNum<<"  "<<B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - ftsPstVel.row(legNum))<<endl;
                cout<<"K__attach_"<<(int)legNum<<"  "<<K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - ftsPstPos.row(legNum))<<endl;
                break;
            }
        }
    }
    xc_dot =  ftsPstVel + xc_dotdot * (1/impCtlRate);
    xc =  ftsPstPos + 0.5 * (xc_dot * (1/impCtlRate));
    cout<<stepFlag<<endl;
    cout<<"force_\n"<<force.transpose()<<endl;

}

/**
 * @brief 
 * Change parameters ofimp
 * @param mK K of impCtller
 * @param mB B of impCtller
 * @param mM M of impCtller
 * @param mode 
 * 0-stance, 1-swing, 2-detach, 3-attach
 */
void IMPControl::impChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM, int mode)
{
    switch(mode) 
    {
        case 0:
            K_stance = mK; B_stance = mB; M_stance = mM;
            break;
        case 1:
            K_swing = mK; B_swing = mB; M_swing = mM;
            break;
        case 2:
            K_detach = mK; B_detach = mB; M_detach = mM;
            break;
        case 3:
            K_attach = mK; B_attach = mB; M_attach = mM;   
            break; 
    }

}