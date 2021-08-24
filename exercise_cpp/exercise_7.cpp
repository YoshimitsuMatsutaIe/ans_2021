/**
 * @file 
 * @brief 逆運動学を解くシミュレーション．同じディレクトリにEigenディレクトリをおいておく．
 * @author matsuta
 */


# include <iostream>
# include <fstream>
# include <cmath>
# include <Eigen/Core>  // Eigen

const static int N = 3;  // タスク空間の次元
const static int M = 7;  // 配置空間の次元



class BaxterKinematics{
    public:
        double L;  // アーム長さ
        double l;
        double L0;
        double L1;
        double L2;
        double L3;
        double L4;
        double L5;
        double L5;

        void joint_origin_Wo(Eigen::Vector q){

        }
};



int main(){

    Eigen::VectorXd q0 = Eigen::VectorXd::Ones(2, 7);


    return 0;
}