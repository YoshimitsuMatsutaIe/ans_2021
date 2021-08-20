# include <iostream>
# include <fstream>
#include <cmath>
using namespace std;

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
    double step = 0.01;
    int iend = (end - start) / step;
    double x[iend];
    x[0] = 1.0;

    for (int i = 1; i < iend; i++){
        x[i] = x[i-1] + dx(x[i-1], a);
    }



    //csv出力
    ofstream file("example_1_cpp.csv");
    file << "x" << endl;
    for (int i = 0; i < iend; i++){
        file << x[i] << "," << endl;
    }

    cout << "終了しました" <<endl;
    return 0;
}