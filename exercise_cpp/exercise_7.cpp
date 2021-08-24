# include <iostream>
# include <fstream>
# include <cmath>
# include "Eigen/Core"  //ヘッダにおいておく

const static int N = 3;  // システムの次元

Eigen::MatrixXf A=Eigen::MatrixXf::Zero(2,2);
    A(0,0)=2;
    A(1,1)=5;