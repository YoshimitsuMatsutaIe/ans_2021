/**
 * @file 
 * @brief 逆運動学を解くシミュレーション．
 * 同じディレクトリにEigenディレクトリを置き一緒にコンパイルしてください．
 * @author matsuta
 */


# include <iostream>
# include <fstream>
# include <math.h>
# include "Eigen/Core"  // Eigen

const static int N = 3;  // タスク空間の次元
const static int M = 7;  // 配置空間の次元
const static double PI = 3.141592653589793;  // 円周率


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
        double Q_MIN[7];
        double Q_MAX[7];

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
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) - h,
            H + L0;
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_3(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) - 0.707106781186548*L2*cos(q(0))*cos(q(1)) - h,
            H + L0 - L2*sin(q(1));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_4(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) - 0.707106781186548*L2*cos(q(0))*cos(q(1)) - 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_5(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) - 0.707106781186548*L2*cos(q(0))*cos(q(1)) - 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2)));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_6(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) - 0.707106781186548*L2*cos(q(0))*cos(q(1)) - 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2))) + L5*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)));
            return o;
        }
    private:
        Eigen::Matrix<double, 3, 1> joint_origin_7(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) - 0.707106781186548*L2*cos(q(0))*cos(q(1)) - 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2))) + L5*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)));
            return o;
        }
    public:
        Eigen::Matrix<double, 3, 1> joint_origin_GL(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 1> o;
            o << 
            L + 0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) + L6*(0.707106781186548*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + 0.707106781186548*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) + 0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + 0.707106781186548*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))),
            0.707106781186548*L1*sin(q(0)) - 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) - 0.707106781186548*L2*cos(q(0))*cos(q(1)) - 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0))) + 0.707106781186548*L4*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4))) + L6*(-0.707106781186548*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + 0.707106781186548*(((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) - 0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + 0.707106781186548*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))) - h,
            H + L0 - L2*sin(q(1)) - L3*cos(q(1))*cos(q(2)) - L4*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2))) + L5*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1))) + L6*(((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)))*sin(q(5)) + (-sin(q(1))*cos(q(3)) - sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(5)));
            return o;
        }
    public:
        Eigen::Matrix<double, 3, 9> joint_origin_all(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 9> os;
            return os;
        }

    public:
        Eigen::Matrix<double, 3, 7> jacobian_GL(Eigen::Matrix<double, 7, 1> q){
            Eigen::Matrix<double, 3, 7> J;
            J <<
            -0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) - 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) + 0.707106781186548*L3*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0))) - 0.707106781186548*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) - 0.707106781186548*L4*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) + 0.707106781186548*L5*(((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4))) + L6*((0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + (0.707106781186548*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + 0.707106781186548*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) + (0.707106781186548*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + 0.707106781186548*cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + (0.707106781186548*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(3)) - 0.707106781186548*sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))), -0.707106781186548*L2*sin(q(0))*sin(q(1)) - 0.707106781186548*L2*sin(q(1))*cos(q(0)) - 0.707106781186548*L3*sin(q(0))*cos(q(1))*cos(q(2)) - 0.707106781186548*L3*cos(q(0))*cos(q(1))*cos(q(2)) - 0.707106781186548*L4*(sin(q(0))*sin(q(1))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2))) - 0.707106781186548*L4*(sin(q(1))*cos(q(0))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2))) + 0.707106781186548*L5*((sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1))) + 0.707106781186548*L5*((sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1))) + L6*((0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + 0.707106781186548*sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1)))*sin(q(5)) + (0.707106781186548*(sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + 0.707106781186548*sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1)))*sin(q(5)) + (-0.707106781186548*sin(q(0))*sin(q(1))*cos(q(3)) - 0.707106781186548*sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-0.707106781186548*sin(q(1))*cos(q(0))*cos(q(3)) - 0.707106781186548*sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(5))), 0.707106781186548*L3*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0))) + 0.707106781186548*L3*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))) - 0.707106781186548*L4*(sin(q(0))*cos(q(2)) - sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3)) - 0.707106781186548*L4*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(3)) + 0.707106781186548*L5*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4))) + 0.707106781186548*L5*((sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + (sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4))) + L6*((0.707106781186548*(sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) + 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4)))*sin(q(5)) + 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3))*cos(q(5)) + (0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + 0.707106781186548*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + 0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(3))*cos(q(5))), -0.707106781186548*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))) - 0.707106781186548*L4*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))) + 0.707106781186548*L5*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + 0.707106781186548*L5*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + L6*(0.707106781186548*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (0.707106781186548*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - 0.707106781186548*sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(5)) + 0.707106781186548*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (0.707106781186548*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - 0.707106781186548*sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(5))), 0.707106781186548*L5*(-((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4))) + 0.707106781186548*L5*(-((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4))) + L6*((-0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) + 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4)))*sin(q(5)) + (-0.707106781186548*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + 0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4)))*sin(q(5))), L6*((0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*cos(q(5)) + (0.707106781186548*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + 0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*cos(q(5)) - (0.707106781186548*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + 0.707106781186548*cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5)) - (0.707106781186548*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + 0.707106781186548*sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))), 0,
            0.707106781186548*L1*sin(q(0)) + 0.707106781186548*L1*cos(q(0)) + 0.707106781186548*L2*sin(q(0))*cos(q(1)) + 0.707106781186548*L2*cos(q(0))*cos(q(1)) + 0.707106781186548*L3*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2))) - 0.707106781186548*L3*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0))) - 0.707106781186548*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3))) + 0.707106781186548*L4*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + sin(q(0))*cos(q(1))*cos(q(3))) + 0.707106781186548*L5*(((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4))) - 0.707106781186548*L5*(((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + (-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4))) + L6*((0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) + 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + (-0.707106781186548*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) - 0.707106781186548*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(4)))*sin(q(5)) + (0.707106781186548*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) + 0.707106781186548*cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(5)) + (-0.707106781186548*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(3)) + 0.707106781186548*sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(5))), -0.707106781186548*L2*sin(q(0))*sin(q(1)) + 0.707106781186548*L2*sin(q(1))*cos(q(0)) - 0.707106781186548*L3*sin(q(0))*cos(q(1))*cos(q(2)) + 0.707106781186548*L3*cos(q(0))*cos(q(1))*cos(q(2)) - 0.707106781186548*L4*(sin(q(0))*sin(q(1))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2))) + 0.707106781186548*L4*(sin(q(1))*cos(q(0))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2))) + 0.707106781186548*L5*((sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1))) - 0.707106781186548*L5*((sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1))) + L6*((0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + 0.707106781186548*sin(q(0))*sin(q(2))*sin(q(4))*cos(q(1)))*sin(q(5)) + (-0.707106781186548*(sin(q(1))*sin(q(3))*cos(q(0)) - cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) - 0.707106781186548*sin(q(2))*sin(q(4))*cos(q(0))*cos(q(1)))*sin(q(5)) + (-0.707106781186548*sin(q(0))*sin(q(1))*cos(q(3)) - 0.707106781186548*sin(q(0))*sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(5)) + (0.707106781186548*sin(q(1))*cos(q(0))*cos(q(3)) + 0.707106781186548*sin(q(3))*cos(q(0))*cos(q(1))*cos(q(2)))*cos(q(5))), -0.707106781186548*L3*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0))) + 0.707106781186548*L3*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))) + 0.707106781186548*L4*(sin(q(0))*cos(q(2)) - sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3)) - 0.707106781186548*L4*(-sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(0))*cos(q(2)))*sin(q(3)) - 0.707106781186548*L5*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4))) + 0.707106781186548*L5*((sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + (sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4))) + L6*((-0.707106781186548*(sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(4)) - 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(3))*cos(q(4)))*sin(q(5)) - 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(3))*cos(q(5)) + (0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(3))*cos(q(4)) + 0.707106781186548*(sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*sin(q(4)))*sin(q(5)) + 0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(3))*cos(q(5))), 0.707106781186548*L4*((sin(q(0))*sin(q(2)) + sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) + sin(q(3))*cos(q(0))*cos(q(1))) - 0.707106781186548*L4*((sin(q(0))*sin(q(1))*cos(q(2)) - sin(q(2))*cos(q(0)))*cos(q(3)) + sin(q(0))*sin(q(3))*cos(q(1))) - 0.707106781186548*L5*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + 0.707106781186548*L5*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*cos(q(4)) + L6*(-0.707106781186548*(-(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (-0.707106781186548*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) + 0.707106781186548*sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(5)) + 0.707106781186548*(-(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) - sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))*cos(q(4)) + (0.707106781186548*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - 0.707106781186548*sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(5))), -0.707106781186548*L5*(-((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) + (-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4))) + 0.707106781186548*L5*(-((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + (sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4))) + L6*((0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*sin(q(4)) - 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*cos(q(4)))*sin(q(5)) + (-0.707106781186548*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*sin(q(4)) + 0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*cos(q(4)))*sin(q(5))), L6*((-0.707106781186548*((-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*cos(q(3)) - sin(q(3))*cos(q(0))*cos(q(1)))*cos(q(4)) - 0.707106781186548*(-sin(q(0))*cos(q(2)) + sin(q(1))*sin(q(2))*cos(q(0)))*sin(q(4)))*cos(q(5)) + (0.707106781186548*((-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*cos(q(3)) - sin(q(0))*sin(q(3))*cos(q(1)))*cos(q(4)) + 0.707106781186548*(sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(2)))*sin(q(4)))*cos(q(5)) - (-0.707106781186548*(-sin(q(0))*sin(q(2)) - sin(q(1))*cos(q(0))*cos(q(2)))*sin(q(3)) - 0.707106781186548*cos(q(0))*cos(q(1))*cos(q(3)))*sin(q(5)) - (0.707106781186548*(-sin(q(0))*sin(q(1))*cos(q(2)) + sin(q(2))*cos(q(0)))*sin(q(3)) + 0.707106781186548*sin(q(0))*cos(q(1))*cos(q(3)))*sin(q(5))), 0,
            0, -L2*cos(q(1)) + L3*sin(q(1))*cos(q(2)) - L4*(-sin(q(1))*sin(q(3))*cos(q(2)) + cos(q(1))*cos(q(3))) + L5*((sin(q(1))*cos(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(2))*sin(q(4))) + L6*(((sin(q(1))*cos(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(2))*sin(q(4)))*sin(q(5)) + (sin(q(1))*sin(q(3))*cos(q(2)) - cos(q(1))*cos(q(3)))*cos(q(5))), L3*sin(q(2))*cos(q(1)) + L4*sin(q(2))*sin(q(3))*cos(q(1)) + L5*(sin(q(2))*cos(q(1))*cos(q(3))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2))) + L6*((sin(q(2))*cos(q(1))*cos(q(3))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2)))*sin(q(5)) + sin(q(2))*sin(q(3))*cos(q(1))*cos(q(5))), -L4*(-sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))) + L5*(sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2)))*cos(q(4)) + L6*((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(5)) + (sin(q(1))*cos(q(3)) + sin(q(3))*cos(q(1))*cos(q(2)))*sin(q(5))*cos(q(4))), L5*(-(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*sin(q(4)) + sin(q(2))*cos(q(1))*cos(q(4))) + L6*(-(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*sin(q(4)) + sin(q(2))*cos(q(1))*cos(q(4)))*sin(q(5)), L6*(((sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(4)) + sin(q(2))*sin(q(4))*cos(q(1)))*cos(q(5)) - (-sin(q(1))*cos(q(3)) - sin(q(3))*cos(q(1))*cos(q(2)))*sin(q(5))), 0;
            return J;
        }

};



int main(){
    std::cout << "running..." <<std::endl;
    //Eigen::VectorXd q0 = Eigen::VectorXd::Ones(2, 7);

    Eigen::Matrix<double, 7, 1> q0;  //
    q0 <<
    0,
    0,
    0,
    0,
    0,
    0,
    0;
    std::cout << q0 << std::endl;


    return 0;
}