# include <iostream>
# include <fstream>
# include <cmath>


const static int N = 2;  // システムの次元

/**
 * @brief van der pol振動子
 * @param t  時間
 * @param[in] x[2]  状態ベクトル
 * @param[in] dx[2]  状態ベクトルの微分
 * @param[in] K  パラメータ
*/
void VanDerPol(double t, double x[N], double dx[N], double K){
    dx[0] = x[1];
    dx[1] = -K * (std::pow(x[0], 2) - 1) * x[1] - x[0];
}


/**
 * @brief オイラー法
 * @param t  時間
 * @param[in] x[2]  状態ベクトル
 * @param[in] dt  刻み時間
 * @param[in] K パラメータ
*/
void euler_method(double t, double x[N], double dt, double K){
    double dx[2];
    VanDerPol(t, x, dx, K);  // 傾き計算

    for (int i=0; i < N; i++){
        x[i] = x[i] + (dx[i] * dt);
    }
}


/**
 * @brief ルンゲクッタ法
 * @param t  時間
 * @param[in] x[2]  状態ベクトル
 * @param[in] dt  刻み時間
 * @param[in] K パラメータ
*/
void RungeKutta_method(double t, double x[N], double dt, double K){
    double dx[N], k1[N], k2[N], k3[N], k4[N];
    

    // 係数計算
    // k1
    for (int i=0; i < N; i++){
        k1[i] = x[i];
    }
    VanDerPol(t, k1, dx, K);

    // k2
    for (int i=0; i < N; i++){
        k2[i] = 2 * k1[i] * dt/2;
    }
    VanDerPol(t+dt/2, k2, dx, K);

    // k3
    for (int i=0; i < N; i++){
        k3[i] = 2 * k2[i] * dt/2;
    }
    VanDerPol(t+dt/2, k3, dx, K);

    // k4
    for (int i=0; i < N; i++){
        k4[i] = k3[i] * dt;
    }
    VanDerPol(t+dt, k4, dx, K);

    // 状態ベクトルを更新
    for (int i=0; i < N; i++){
        x[i] = x[i] + (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])*dt/6;
    }
}


int main(){

    // 時間に関すtるパラメータ
    double dt = 0.01;  // 刻み時間
    double t_end = 10;  // ケツの時間

    // ダイナミクスのパラメータ
    double K = 1.5;  // ヴァンデルローストのパラメータ

    // 初期値
    double x0[N] = {0.1, 0.1};


    // 数値シミュレーション実行
    std::cout << "数値シミュレーション実行中..." << std::endl;

    double x[N];  // 状態ベクトル
    std::ofstream file("exercise_2_cpp_data.csv");
    file << "t, x, v" << std::endl;
    for (int i=0; i < N; i++){
        x[i] = x0[i];
    }  // 初期化
    for (double t=0; t < t_end+dt; t+=dt){
        RungeKutta_method(t, x, dt, K);
        file << t;
        for (int i=0; i < N; i++){
            file << "," << x[i];
        }
        file << std::endl;
    }

}