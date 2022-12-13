#include "robotMotionControl.h"

#define ForceLPF  0.9
#define StepHeight  (30.0/1000.0)
#define TimeHeight (2.0/4.0)  // time for trajectory within vertical part
#define swingVelFactor 3      

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
/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
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
/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */
void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresPos = initPosition;
    legCmdPos = initPosition;
    initFootPos = initPosition;
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
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void MotionControl::updatejointPresPos(vector<float> jointPos)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPresPos(i,j) = jointPos[i*3 + j];
    }
}
/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPresVel(4,3)
 */
void MotionControl::updatejointPresVel(vector<float> jointVel)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPresVel(i,j) = jointVel[i*3 + j];
    }
}
/**
 * @brief 
 * Calculcate jacobian_vector with jointPresPos
 */
void MotionControl::updateJacobians()
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
    {
        float th0,th1,th2;
        float factor_y, factor_x, factor_z, factor_s12, factor_s0;  
        th0 =  jointPresPos(legNum,0);
        th1 =  jointPresPos(legNum,1);
        th2 =  jointPresPos(legNum,2); 
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
/**
 * @brief 
 * update Vel of feet in shoulder coordinate
 */
void MotionControl::updateFtsPresVel()
{
    Matrix <float, 3, 1> temp_vel;
    for(int i=0; i<4; i++)
    {
        temp_vel = jacobian_vector[i] * jointPresVel.row(i).transpose();
        legPresVel.row(i) = temp_vel.transpose();
    }
}
/**
 * @brief forwardKinematics
 * 
 * @param mode 
 * if mode=0    calculcate foot position(legCmdPos) with jointCmdPos(target),
 * if mode=1    calculcate foot position(legPresPos) with jointPresPos(present) and update legPos_last
 */
void MotionControl::forwardKinematics(int mode)
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
    {
        float th0,th1,th2;
        float factor_y, factor_x,  factor_z, factor_s12, factor_s0;  
        if(mode==0)
        {
            th0 =  jointCmdPos(legNum,0);
            th1 =  jointCmdPos(legNum,1);
            th2 =  jointCmdPos(legNum,2);    
        }
        else if(mode==1)
        {
            th0 =  jointPresPos(legNum,0);
            th1 =  jointPresPos(legNum,1);
            th2 =  jointPresPos(legNum,2);     
        }
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
            legPos_last = legPresPos;
            legPresPos(legNum,0) = factor_x * (-factor_s12 * L1 * sin(th1) + L2 * cos(th1 + th2));
            legPresPos(legNum,1) = factor_y * ((( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * cos(th0) + factor_s0 * L3 * sin(th0)));
            legPresPos(legNum,2) = factor_z * (( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * factor_s0 * sin(th0) -  L3 * cos(th0));
        }
    }

}
/**
 * @brief inverse Kinematics
 * 
 * @param cmdpos 
 * Calculcate joint angles (jointCmdPos) for motors with foot position(cmdpos) in shoulder coordinate
 */
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
            jointCmdPos(legNum,0) = factor_xc * (asin(L3 / sqrt( cmdpos(legNum,2)*cmdpos(legNum,2) + cmdpos(legNum,1)*cmdpos(legNum,1) )) + atan2(cmdpos(legNum,2),factor_y * cmdpos(legNum,1)) );     
            jointCmdPos(legNum,1) = factor_yc * (asin((cmdpos(legNum,1) * cmdpos(legNum,1) + cmdpos(legNum,0) * cmdpos(legNum,0) + cmdpos(legNum,2) * cmdpos(legNum,2) + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (cmdpos(legNum,1) * cmdpos(legNum,1) +  cmdpos(legNum,0) * cmdpos(legNum,0) + cmdpos(legNum,2) * cmdpos(legNum,2) - L3 * L3)))
                    - atan2(sqrt(cmdpos(legNum,1) * cmdpos(legNum,1) + cmdpos(legNum,2) * cmdpos(legNum,2) - L3 * L3) , factor_x * cmdpos(legNum,0)));
            jointCmdPos(legNum,2) = factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - cmdpos(legNum,1) * cmdpos(legNum,1) - cmdpos(legNum,0) * cmdpos(legNum,0) - cmdpos(legNum,2) * cmdpos(legNum,2)) / (2 * L1 * L2));

        }
}

void MotionControl::nextStep()
{

    if (abs(timePresent - timeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
        // legCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(timeForStancePhase(legNum,0) < timeForStancePhase(legNum,1))
        {
            if(timePresent > timeForStancePhase(legNum,0) - timePeriod/2 && timePresent < timeForStancePhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                stepFlag[legNum] = stance;
            else    //swing phase 
                stepFlag[legNum] = swing;            
        }
        else
        {
            if(timePresent > timeForStancePhase(legNum,0) - timePeriod/2 || timePresent < timeForStancePhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                stepFlag[legNum] = stance;
            else    //swing phase 
                stepFlag[legNum] = swing;
        }
        if(stepFlag[legNum] == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
                targetCoMPosition(legNum, pos) += targetCoMVelocity(pos) * timePeriod;
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
                // shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                // shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForStancePhase(legNum,1)) < timePeriod + 1e-4)  // if on the end pos   
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
        }
        else    //swing phase 
        {
            Matrix<float, 1, 3> swingPhaseVelocity = -(stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / timeForSwing(legNum) ;
            float x, xh, m, n, k;
            //cout<<"legNum_"<<(int)legNum<<":"<<swingPhaseVelocity.array()<<"  ";
            if( ( timePresentForSwing(legNum) - timeForSwing(legNum) * TimeHeight ) < 1e-4 && timePresentForSwing(legNum) > -1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    legCmdPos(legNum, pos) += swingPhaseVelocity(pos) * timePeriod * swingVelFactor;     // for vertical down
                x = legCmdPos(legNum, 0) - stancePhaseEndPos(legNum, 0);
                xh = -(stancePhaseEndPos(legNum, 0) - stancePhaseStartPos(legNum, 0)) * TimeHeight * swingVelFactor;

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = StepHeight / xh / xh;
                legCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + stancePhaseEndPos(legNum, 2);
            }
            else if( timePresentForSwing(legNum) - timeForSwing(legNum) < 1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {
                for(uint8_t pos=0; pos<2; pos++)
                    legCmdPos(legNum, pos) += swingPhaseVelocity(pos) * timePeriod * (1 - swingVelFactor * TimeHeight) / (1 - TimeHeight); // targetCoMVelocity
                legCmdPos(legNum, 2) -= StepHeight / timeForSwing(legNum) / (1 - TimeHeight) * timePeriod;
            }  

            if(swingPhaseVelocity( 0, 0) == 0)      //first step
            {
                if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) < 1e-4 && timePresentForSwing(legNum) > -1e-4)
                    legCmdPos(legNum, 2) += StepHeight / timeForSwing(legNum) * 2 * timePeriod;
                if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) > -1e-4)
                    legCmdPos(legNum, 2) -=  StepHeight / timeForSwing(legNum) * 2 * timePeriod;
            }
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
    }

    
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(stepFlag[legNum] != stance) timePresentForSwing(legNum) += timePeriod;
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
    // string2float("../include/imp_parameter.csv",impdata);   // 0-stance, 1-swing, 2-detach, 3-attach
    string2float("../include/adm_parameter.csv",impdata);   // adm
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
 * Calculcate feedback force in LPF with jacobian for impCtller.
 * @note Feedback force is driving force. Use the counter force to represent the driving force. driving force = - counter force
 * @param torque update present force with present_torque
 */
void IMPControl::updateFtsPresForce(vector<float> torque)
{
    Matrix<float, 3, 4> temp;
    if(force(2,3) - force_last(2,3) > 0.3 || force(2,3) - force_last(2,3) < -0.3)
        temp.setZero();
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = torque[i+j*3];
    for (int i=0; i<4; i++)
        force.col(i) = ForceLPF * force_last.col(i) + (1-ForceLPF) * jacobian_vector[i].transpose().inverse() * temp.col(i);
    force_last = force;
}
/**
 * @brief 
 * Calculcate target torque with force and jacobin.
 * @param force 
 */
void IMPControl::updateTargTor(Matrix<float, 3, 4> force)
{
    for (int i=0; i<4; i++)
        target_torque.col(i) = jacobian_vector[i] * force.col(i);
}
/**
 * @brief 
 * Deliver parameters in nextstep() to impCtller.
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
        if(stepFlag[legNum] == swing) //swing
        {
            if( ( timePresentForSwing(legNum) - (timeForSwing(legNum) - timePeriod *8) ) > -1e-4 )
            {
                stepFlag[legNum] = attach;   //attach
                target_force.row(legNum) << 0, 0, -1.6;  // x, y, z
            }
            else if( ( timePresentForSwing(legNum) - timePeriod *8 ) < 1e-4 && timePresentForSwing(legNum) > 1e-4)
            {
                stepFlag[legNum] = detach;   //detach
                target_force.row(legNum) << 0, 0, 1.5;
            }
            else    //swing
            {
                stepFlag[legNum] = swing;
                target_force.row(legNum) << 0, 0, 0;
            }
        }
        else        //stance
        {
            stepFlag[legNum] = stance;
            target_force.row(legNum) << -0.6, 0, -1.6;    
        }
    }

}
/**
 * @brief 
 * Calculcate legCmdPos with target_acc, target_force, target_vel, target_pos and force,
 * or calculcate target force with target_acc, target_force, target_vel, target_pos and legPresPos.
 * 
 * @param mode 0 for Impedance control;    1 for Admittance control
 */
void IMPControl::impCtller(int mode)
{
    if(mode == 0)  // Impedance control
    {
        xc_dot = (legPresPos - legPos_last) * impCtlRate;
        xc_dotdot = (legPresVel - xc_dot) * impCtlRate;
        for(uint8_t legNum=0; legNum<4; legNum++)
        {
            force.transpose().row(legNum) = target_force.row(legNum)  
            + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
            + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) ;
            // + M_stance.row(legNum).cwiseProduct(target_acc.row(legNum) - xc_dotdot.row(legNum));
            cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<endl;
            cout<<"K__stance_"<<(int)legNum<<"  "<<K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
            cout<<"B__stance_"<<(int)legNum<<"  "<<B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
            // cout<<"force"<<force.transpose().row(legNum)<<endl;
        }
        updateTargTor(force);
        // cout<<"target_pos:"<<endl<<target_pos<<endl;
        // cout<<"legPresPos:"<<endl<<legPresPos<<endl;
        cout<<"force:"<<endl<<force<<endl;
        cout<<"target_torque:"<<endl<<target_torque<<endl<<endl;
    }
    else if(mode == 1)  // Admittance control    xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    {
        for(uint8_t legNum=0; legNum<4; legNum++)
        {   
            // xc_dotdot.row(legNum) =  target_acc.row(legNum) 
            // + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
            // + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
            // + M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
            // cout<<"K__attach_"<<(int)legNum<<"  "<<K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
            // cout<<"B__attach_"<<(int)legNum<<"  "<<B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
            // cout<<"M__attach_"<<(int)legNum<<"  "<<M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;      
            switch(stepFlag[legNum])
            {
                case 0: //stance
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    // cout<<"K__stance_"<<(int)legNum<<"  "<<K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    // cout<<"B__stance_"<<(int)legNum<<"  "<<B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    // cout<<"M__stance_"<<(int)legNum<<"  "<<M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                    break;

                case 1: //swing
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_swing.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_swing.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_swing.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );

                    break;

                case 2: //detach
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))   
                    + M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    cout<<"K__detach_"<<(int)legNum<<"  "<<K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    cout<<"B__detach_"<<(int)legNum<<"  "<<B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    cout<<"M__detach_"<<(int)legNum<<"  "<<M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                    break;

                case 3: //attach
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    cout<<"K__attach_"<<(int)legNum<<"  "<<K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    cout<<"B__attach_"<<(int)legNum<<"  "<<B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    cout<<"M__attach_"<<(int)legNum<<"  "<<M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl<<endl;                 
                    break;
            }
            // cout<<stepFlag[legNum]<<endl;
        }
        xc_dot =  legPresVel + xc_dotdot * (1/impCtlRate);
        xc =  legPresPos + (xc_dot * (1/impCtlRate));
        
        // cout<<"force_\n"<<force.transpose()<<endl;
    }

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