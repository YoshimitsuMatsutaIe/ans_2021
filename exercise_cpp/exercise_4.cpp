#include <iostream>
#include <fstream>
#include <math.h>
//#include "../exercise_cpp/Eigen/Core"

const static int N = 3;  // システムの次元

/**
    @brief  バネマスダンパのパラメータ
*/
struct Param_SpringMassDamperModel {
    double m;  // 質量
    double c;  // 摩擦係数
    double k;  // ばね定数
    double g;  // 重力加速度
};

/**
    @brief  PIDのパラメータ
*/
struct Param_PID {
    double p;  // 比例ゲイン
    double i;  // 積分ゲイン
    double d;  // 微分ゲイン
};

/**
    @brief  PIDのゲインレンジ
*/
struct range_PID {
    double p_max;  // 比例ゲインの最大値
    double p_min;  // 比例ゲインの最小値
    double i_max;  // 積分ゲインの最大値
    double i_min;  // 積分ゲインの最小値
    double d_max;  // 微分ゲインの最大値
    double d_min;  // 微分ゲインの最小値
};

/**
 * @brief 状態方程式
 * @param t  時間
 * @param[in] x[2]  状態ベクトル
 * @param[in] dx[2]  状態ベクトルの微分
 * @param[in] K  パラメータ
 * @param[in] xg  目標値
*/
void Dynamics(double t, double x[N], double dx[N], Param_SpringMassDamperModel p, Param_PID K, double xg){
    dx[0] = x[1];
    dx[1] = (-p.c*x[1] -p.k*x[0] + x[2]) / p.m;
    dx[2] = -K.p*x[1] + K.i*(xg - x[0]) - K.d*dx[1];
}


/**
 * @brief ルンゲクッタ法
 * @param t  時間
 * @param[in] x[2]  状態ベクトル
 * @param[in] dt  刻み時間
 * @param[in] K パラメータ
*/
void RungeKutta_method(double t, double x[N], double dt, Param_SpringMassDamperModel p, Param_PID K, double xg){
    double dx[N], k1[N], k2[N], k3[N], k4[N];
    
    // 係数計算
    // k1
    for (int i=0; i < N; i++){
        k1[i] = x[i];
    }
    Dynamics(t, k1, dx, p, K, xg);

    // k2
    for (int i=0; i < N; i++){
        k2[i] = 2 * k1[i] * dt/2;
    }
    Dynamics(t+dt/2, k2, dx, p, K, xg);

    // k3
    for (int i=0; i < N; i++){
        k3[i] = 2 * k2[i] * dt/2;
    }
    Dynamics(t+dt/2, k3, dx, p, K, xg);

    // k4
    for (int i=0; i < N; i++){
        k4[i] = k3[i] * dt;
    }
    Dynamics(t+dt, k4, dx, p, K, xg);

    // 状態ベクトルを更新
    for (int i=0; i < N; i++){
        x[i] = x[i] + (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])*dt/6;
    }
}

double calc_gain(double K_init, double K_end, double n, double n_now){
    return K_init + (K_end - K_init) / n * n_now;
}

/**
 * @brief アニメーションのデータ作成
 * @param[in] rangeK  ゲインの幅
 * @param[in] n  アニメーションの枚数
 * @param[in] p  バネマスダンパのパラメータ
*/
void do_2(double dt, double t_end, range_PID rangeK, double n, Param_SpringMassDamperModel p, double xg, double x_init, double v_init){

    double x[N];  // 状態ベクトル
    Param_PID K;  // ゲイン

    // 数値シミュレーション実行
    std::cout << "数値シミュレーション実行中..." << std::endl;
    std::ofstream file("exercise_4_cpp_data.csv");

    file << "t, x, v, u, Kp, Ki, Kd" << std::endl;
    for (int j=0; j < n; j++){
        
        K = {
            calc_gain(rangeK.p_min, rangeK.p_max, n, j),
            calc_gain(rangeK.i_min, rangeK.i_max, n, j),
            calc_gain(rangeK.d_min, rangeK.d_max, n, j),
            };
        // 初期値を代入
        x[0] = x_init;
        x[1] = v_init;
        x[2] = K.p*(xg - x[0]) + K.d*(0.0 - v_init);

        for (double t=0; t < t_end+dt; t+=dt){
            RungeKutta_method(t, x, dt, p, K, xg);
            file << t;
            for (int i=0; i < N; i++){
                file << "," << x[i];
            }
            file << "," << K.p;
            file << "," << K.i;
            file << "," << K.d;
            file << std::endl;
        }
    }
}


int main(){
    std::cout << "running..." << std::endl;
    // 時間に関すtるパラメータ
    double dt = 0.01;  // 刻み時間
    double t_end = 10;  // ケツの時間

    // バネマスダンパのパラメータ
    Param_SpringMassDamperModel param_basic;
    param_basic = {1.0, 1.0, 1.0, 9.8};

    range_PID K;  // PIDのゲイン幅
    K = {5.0, 4.5, 5.0, 4.5, 0.05, 0.0};
    int n = 50;  // アニメーションの枚数

    double xg = 1.0;  // 目標位置

    // 初期値に関するもの
    double x0 = 0.0;  // 初期位置
    double v0 = 0.0;  // 初期測度

    do_2(dt, t_end, K, n, param_basic, xg, x0, v0);
    
    std::cout << "done." << std::endl;
    return 0;
}