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

class IMPControl
{
    public:
        Matrix<float, 4, 3> target_pos; // LF RH LH RH ; x y z  in CoM cordinate 
        Matrix<float, 4, 3> target_vel;
        Matrix<float, 4, 3> target_acc; // Force in target position
        Matrix<float, 4, 3> xc_dotdot;
        Matrix<float, 4, 3> xc_dot;
        Matrix<float, 4, 3> xc;
        Matrix<float, 3, 4> force;              //x y z ; LF RH LH RH
        Matrix<float, 4, 1> K;                     //LF RH LH RH
        Matrix<float, 4, 1> B;
        Matrix<float, 4, 1> M;

        void impCtller( );
        IMPControl();
};