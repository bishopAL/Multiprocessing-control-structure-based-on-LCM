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
        Matrix<float, 4, 3> target_pos;
        Matrix<float, 4, 3> target_vel;
        Matrix<float, 4, 3> target_acc;
        Matrix<float, 4, 3> xc_dotdot;
        Matrix<float, 4, 3> xc_dot;
        Matrix<float, 4, 3> xc;
        Matrix<float, 3, 4> force;
        Matrix<float, 4, 1> K;
        Matrix<float, 4, 1> B;
        Matrix<float, 4, 1> M;

        void impCtller( );
        IMPControl();
};