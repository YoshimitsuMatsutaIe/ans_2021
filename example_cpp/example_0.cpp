#include <iostream>
#include <math.h>


double QuadraticFunc(double x){
    double y = std::pow(x, 2.0) + x + 1;
    return y;
}

int main(){
    std::cout << "running..." << std::endl;

    double start = 0;  // 開始
    double end = 10;  // 終わり
    double step = 0.1;  // 刻み幅
    int iend;
    iend = (end - start) / step;
    double x = start;
    double y;

    double y_list[iend];

    for (int i = 0; i < iend+1; i++){
        x = x + step;
        y = QuadraticFunc(x);
        std::cout << y << " , ";
        y_list[i] = y;
    }


    return 0;
}