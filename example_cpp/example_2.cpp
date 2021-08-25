# include <iostream>
# include <fstream>
#include <cmath>
using std::cout;
using std::endl;


double dx(double x, double a){
    //微分方程式
    double y = a * x;
    return y;
}


int main(){
    cout << "実行中..." << endl;

    double a = 2.0;

    double start = 0;
    double end = 3;
    const int n = 100000;
    double step = (start - end) / n;
    double x[n];
    x[0] = 1.0;  // 初期値代入

    for (int i = 1; i < n; i++){
        x[i] = x[i-1] + dx(x[i-1], a) * step;
    }


    //csv出力
    std::ofstream file("example_1_cpp.csv");
    file << "x" << endl;
    for (int i = 0; i < n; i++){
        file << x[i] << "," << endl;
    }

    cout << "終了しました" <<endl;
    return 0;
}