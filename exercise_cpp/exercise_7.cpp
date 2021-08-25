/**
 * @file 
 * @brief 逆運動学を解くシミュレーション．
 * @author matsuta
 * @note Eigenが必要（今の所ビルドできない．原因不明）
 */


#include <iostream>
#include <fstream>
#include <math.h>
//#include "Eigen/Core"  // Eigen
#include "Eigen/Dense"

// int main(){
//     std::cout << "OK" << std::endl;
//     return 0;
// }



const static double PI = 3.141592653589793;  // 円周率


/**
 * @class BaxterKinematics
 * @brief baxterロボットの順運動学を計算
 */
class BaxterKinematics{

    private:
        double L = 278e-3;
        double h = 64e-3;
        double H = 1104e-3;
        double L0 = 270.35e-3;
        double L1 = 69e-3;
        double L2 = 364.35e-3;
        double L3 = 69e-3;
        double L4 = 374.29e-3;
        double L5 = 10e-3;
        double L6 = 368.3e-3;

    public:
        Eigen::Matrix<double, 7, 1> Q_MIN;  // 実現可能な関節角度の最大値
        Eigen::Matrix<double, 7, 1> Q_MAX;  // 実現可能な関節角度の最大値
        Eigen::Matrix<double, 7, 1> Q_INIT;  // ロボットの初期姿勢

    private:
        Eigen::Matrix<double, 3, 1> joint_origin_Wo(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            0,
            0,
            0;
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_BL(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L,
            -h,
            H;
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_0(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L,
            -h,
            H + L0;
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_1(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L,
            -h,
            H + L0;
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_2(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) - h,
            H + L0;
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_3(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) - cos(PI/4)*L2*cos(q(0))*cos(q(1)) - h,
            H + L0 - L2*sin(q(1));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_4(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) - cos(PI/4)*L2*cos(q(0))*cos(q(1)) - cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_5(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) - cos(PI/4)*L2*cos(q(0))*cos(q(1)) - cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2)));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_6(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) - cos(PI/4)*L2*cos(q(0))*cos(q(1)) - cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2))) + L5*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_7(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) - cos(PI/4)*L2*cos(q(0))*cos(q(1)) - cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2))) + L5*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)));
            return o;
        }
    public:
        Eigen::Matrix<double, 3, 1> joint_origin_GL(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) + L6*(cos(PI/4)*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + cos(PI/4)*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) + cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + cos(PI/4)*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))),
            cos(PI/4)*L1*sin(q(0)) - cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) - cos(PI/4)*L2*cos(q(0))*cos(q(1)) - cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + cos(PI/4)*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) + L6*(-cos(PI/4)*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + cos(PI/4)*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) - cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + cos(PI/4)*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2))) + L5*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1))) + L6*(((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)))*sin(q(5)) + (-sin(q(1))*cos(q(3)) - sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(5)));
            return o;
        }
    // public:
    //     Eigen::Matrix<double, 3, 9> joint_origin_all(Eigen::Matrix<double, 7, 1> q){
    //         Eigen::Matrix<double, 3, 9> os;
    //         return os;
    //     }

    public:
        Eigen::Matrix<double, 3, 7> jacobian_GL(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 7> J;
            J <<
            -cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) - cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + cos(PI/4)*L3*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0))) - cos(PI/4)*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - cos(PI/4)*L4*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + cos(PI/4)*L5*(((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4))) + L6*((cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + (cos(PI/4)*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + cos(PI/4)*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) + (cos(PI/4)*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(PI/4)*cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + (cos(PI/4)*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(3)) - cos(PI/4)*sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))), -cos(PI/4)*L2*sin(q(0))*sin(q(1)) - cos(PI/4)*L2*sin(q(1))*cos(q(0)) - cos(PI/4)*L3*sin(q(0))*cos(q(1))*cos(q(2)) - cos(PI/4)*L3*cos(q(0))*cos(q(1))*cos(q(2)) - cos(PI/4)*L4*(sin(q(0))*sin(q(1))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2))) - cos(PI/4)*L4*(sin(q(1))*cos(q(0))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2))) + cos(PI/4)*L5*((sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1))) + cos(PI/4)*L5*((sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1))) + L6*((cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + cos(PI/4)*sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1)))*sin(q(5)) + (cos(PI/4)*(sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + cos(PI/4)*sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1)))*sin(q(5)) + (-cos(PI/4)*sin(q(0))*sin(q(1))*cos(q(3)) - cos(PI/4)*sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-cos(PI/4)*sin(q(1))*cos(q(0))*cos(q(3)) - cos(PI/4)*sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(5))), cos(PI/4)*L3*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0))) + cos(PI/4)*L3*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))) - cos(PI/4)*L4*(sin(q(0))*cos(q(2)) - sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3)) - cos(PI/4)*L4*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(3)) + cos(PI/4)*L5*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4))) + cos(PI/4)*L5*((sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + (sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4))) + L6*((cos(PI/4)*(sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) + cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4)))*sin(q(5)) + cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3))*cos(q(5)) + (cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + cos(PI/4)*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(3))*cos(q(5))), -cos(PI/4)*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))) - cos(PI/4)*L4*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))) + cos(PI/4)*L5*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + cos(PI/4)*L5*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + L6*(cos(PI/4)*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (cos(PI/4)*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - cos(PI/4)*sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(5)) + cos(PI/4)*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (cos(PI/4)*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - cos(PI/4)*sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(5))), cos(PI/4)*L5*(-((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4))) + cos(PI/4)*L5*(-((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4))) + L6*((-cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) + cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4)))*sin(q(5)) + (-cos(PI/4)*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4)))*sin(q(5))), L6*((cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*cos(q(5)) + (cos(PI/4)*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*cos(q(5)) - (cos(PI/4)*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(PI/4)*cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5)) - (cos(PI/4)*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + cos(PI/4)*sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))), 0,
            cos(PI/4)*L1*sin(q(0)) + cos(PI/4)*L1*cos(q(0)) + cos(PI/4)*L2*sin(q(0))*cos(q(1)) + cos(PI/4)*L2*cos(q(0))*cos(q(1)) + cos(PI/4)*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) - cos(PI/4)*L3*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0))) - cos(PI/4)*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) + cos(PI/4)*L4*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3))) + cos(PI/4)*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) - cos(PI/4)*L5*(((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4))) + L6*((cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + (-cos(PI/4)*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) - cos(PI/4)*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) + (cos(PI/4)*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(PI/4)*cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + (-cos(PI/4)*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(3)) + cos(PI/4)*sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))), -cos(PI/4)*L2*sin(q(0))*sin(q(1)) + cos(PI/4)*L2*sin(q(1))*cos(q(0)) - cos(PI/4)*L3*sin(q(0))*cos(q(1))*cos(q(2)) + cos(PI/4)*L3*cos(q(0))*cos(q(1))*cos(q(2)) - cos(PI/4)*L4*(sin(q(0))*sin(q(1))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2))) + cos(PI/4)*L4*(sin(q(1))*cos(q(0))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2))) + cos(PI/4)*L5*((sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1))) - cos(PI/4)*L5*((sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1))) + L6*((cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + cos(PI/4)*sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1)))*sin(q(5)) + (-cos(PI/4)*(sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) - cos(PI/4)*sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1)))*sin(q(5)) + (-cos(PI/4)*sin(q(0))*sin(q(1))*cos(q(3)) - cos(PI/4)*sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(5)) + (cos(PI/4)*sin(q(1))*cos(q(0))*cos(q(3)) + cos(PI/4)*sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(5))), -cos(PI/4)*L3*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0))) + cos(PI/4)*L3*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))) + cos(PI/4)*L4*(sin(q(0))*cos(q(2)) - sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3)) - cos(PI/4)*L4*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(3)) - cos(PI/4)*L5*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4))) + cos(PI/4)*L5*((sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + (sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4))) + L6*((-cos(PI/4)*(sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) - cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4)))*sin(q(5)) - cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3))*cos(q(5)) + (cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + cos(PI/4)*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(3))*cos(q(5))), cos(PI/4)*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))) - cos(PI/4)*L4*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))) - cos(PI/4)*L5*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + cos(PI/4)*L5*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + L6*(-cos(PI/4)*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (-cos(PI/4)*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) + cos(PI/4)*sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(5)) + cos(PI/4)*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (cos(PI/4)*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - cos(PI/4)*sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(5))), -cos(PI/4)*L5*(-((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4))) + cos(PI/4)*L5*(-((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4))) + L6*((cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) - cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4)))*sin(q(5)) + (-cos(PI/4)*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4)))*sin(q(5))), L6*((-cos(PI/4)*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) - cos(PI/4)*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*cos(q(5)) + (cos(PI/4)*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + cos(PI/4)*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*cos(q(5)) - (-cos(PI/4)*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(PI/4)*cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5)) - (cos(PI/4)*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + cos(PI/4)*sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))), 0,
            0, -L2*cos(q(1)) + L3*sin(q(1))*cos(q(2)) - L4*(-sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3))) + L5*((sin(q(1))*cos(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(2))*sin(q(4))) + L6*(((sin(q(1))*cos(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(2))*sin(q(4)))*sin(q(5)) + (sin(q(1))*sin(q(3))*cos(q(2)) - cos(q(1))*cos(q(3)))*cos(q(5))), L3*sin(q(2))*cos(q(1)) + L4*sin(q(2))*sin(q(3))*cos(q(1)) + L5*(sin(q(2))*cos(q(1))*cos(q(3))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2))) + L6*((sin(q(2))*cos(q(1))*cos(q(3))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2)))*sin(q(5)) + sin(q(2))*sin(q(3))*cos(q(1))*cos(q(5))), -L4*(-sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))) + L5*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(4)) + L6*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(5)) + (sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2)))*sin(q(5))*cos(q(4))), L5*(-(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*sin(q(4)) + sin(q(2))*cos(q(1))*cos(q(4))) + L6*(-(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*sin(q(4)) + sin(q(2))*cos(q(1))*cos(q(4)))*sin(q(5)), L6*(((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)))*cos(q(5)) - (-sin(q(1))*cos(q(3)) - sin(q(3))*cos(q(1))*cos(q(2)))*sin(q(5))), 0;
            return J;
        }

};


/**
 * @class InvKinematics
 * @brief 逆運動学を計算する
 */
class InvKinematics : public BaxterKinematics {
    public:
        Eigen::Matrix<double, 7, 1> calc_desired_joint_angle(Eigen::Matrix<double, 3, 1> xg, double alpha){
            Eigen::Matrix<double, 7, 1> qd;  // 所望の関節角度
            Eigen::Matrix<double, 7, 1> q;  // 関節角度
            Eigen::Matrix<double, 7, 1> dq;  // 関節角度の増分
            Eigen::Matrix<double, 3, 7> J;  // エンドエフェクタ位置のヤコビ行列
            Eigen::Matrix<double, 3, 1> x;  // エンドエフェクタ位置
            Eigen::Matrix<double, 3, 1> dx;  // エンドエフェクタ位置と目標位置の誤差ベクトル

            double e;  // エンドエフェクタと目標位置との誤差
            double error_t = 0.0001; // 誤差の許容値

            int n_trial = 10;  // 試行回数の最大値

            for (int n=0; n < n_trial; n++){
                q = Eigen::Matrix<double, 7, 1>::Random(7, 1);
                q = q * PI/180;  // radianに変換
                x = joint_origin_GL(q);

                for (int i=0; i < 10000; i++){
                    dx = xg - x;
                    e = dx.norm();
                    if (e < error_t && ((Q_MAX - q).array()>0.0).all() && ((q - Q_MIN).array()>0.0).all()){
                        return q;
                    }
                    else{
                        J = jacobian_GL(q);
                        dq = J.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dx);
                        q += alpha * dq;
                    }
                    
                }
            }
            return Q_INIT;
        }
};


int main(){
    std::cout << "running..." <<std::endl;

    Eigen::Matrix<double, 3, 1> xd;  // 所望のエンドエフェクタ位置
    xd <<
    0.3,
    -0.6,
    1;
    double alpha = 0.5;  // 重み

    Eigen::Matrix<double, 7, 1> qd;  // 所望の関節角度
    
    InvKinematics kinem;
    kinem.Q_MIN <<
    -141 * PI / 180,
    -123 * PI / 180,
    -173 * PI / 180,
    -3 * PI / 180,
    -175 * PI / 180,
    -90 * PI / 180,
    -175 * PI / 180;
    kinem.Q_MAX <<
    51 * PI / 180,
    60 * PI / 180,
    173 * PI / 180,
    150 * PI / 180,
    175 * PI / 180,
    120 * PI / 180,
    175 * PI / 180;
    kinem.Q_INIT <<
    0 * PI / 180,
    -31 * PI / 180,
    0 * PI / 180,
    43 * PI / 180,
    0 * PI / 180,
    72 * PI / 180,
    0 * PI / 180;

    qd = kinem.calc_desired_joint_angle(xd, alpha);

    std::cout << qd << std::endl;


    // Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
    // std::cout << "Here is the matrix A:\n" << A << std::endl;
    // Eigen::VectorXf b = Eigen::VectorXf::Random(3);
    // std::cout << "Here is the right hand side b:\n" << b << std::endl;
    // std::cout << "The least-squares solution is:\n"
    //     << A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << std::endl;

    return 0;
}