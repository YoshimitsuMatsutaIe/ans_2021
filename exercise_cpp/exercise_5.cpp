#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include "Eigen/Dense"


using std::cout;
using std::endl;


/**
 * @brief リカッチ方程式を解く
 * @param[in] A  A
 * @param[in] B  B
 * @param[in] Q  状態に関する重み行列
 * @param[in] R  入力に関する重み行列
*/
// Eigen::MatrixXd RiccatiSolver(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R){
    
//     int n = (int)A.rows();
    
//     Eigen::MatrixXd H(2*n, 2*n);  //ハミルトン行列
//     H <<
//     A, -B*R.inverse()*B.transpose(),
//     -Q, -A.transpose();
    
//     Eigen::EigenSolver < Eigen::Matrix <double, 4, 4>> es(H);  //固有値と固有ベクトル
//     // if (es.info() != Eigen::Success){
//     //     abort();
//     // }
    
//     std::vector <Eigen::VectorXd> v_set, u_set;
//     for(int i=0; i < 2*n; i++){
//         if (es.eigenvalues().real()(i) < 0){
//             v_set.push_back(es.eigenvectors().real().block(0,i,n,1) );
//             u_set.push_back(es.eigenvectors().real().block(n,i,n,1) );
//         }
//     }
    
//     int num = (int)v_set.size();

//     Eigen::MatrixXd v(n,num), u(n,num);
//     for(int i=0; i < num; i++){
//         v.block(0,i,n,1) = v_set[i];
//         u.block(0,i,n,1) = u_set[i];
//     }
    
//     //解Pを求める
//     Eigen::MatrixXd P = u * v.inverse();
//     return P;
// }

int main(int argc, const char * argv[]) {
    Eigen::MatrixXd A(3,3), B(3,1), Q(3,3), R(1,1);
    A <<
    1.1, 2.0, 3.0,
    0, 0.95, 1.20,
    1.2, 0.01, 10.5;
    
    B <<
    1.0,
    0.0,
    0.847;

    Q <<
    1000, 0, 0,
    0, 1000, 0,
    0, 0, 1000;

    R <<
    1;

    Eigen::MatrixXd P = RiccatiSolver(A, B, Q, R);
    cout << "P = " << endl << P << endl;
    return EXIT_SUCCESS;
}



