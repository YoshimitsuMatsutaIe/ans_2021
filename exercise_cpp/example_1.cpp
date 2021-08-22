# include <iostream>
# include <fstream>
#include <cmath>
using namespace std;


double QuadraticFunc(double x){
    double y = std::pow(x, 2.0) + x + 1;
    return y;
}

int main(){
    cout << "value..." << endl;

    double start = 0;
    double end = 10;
    double step = 0.1;
    int iend = (end - start) / step;
    double x = start;
    double y;

    double y_list[iend];
    

    for (int i = 0; i < iend+1; i++){
        x = x + step;
        y = QuadraticFunc(x);
        cout << y << " , ";
        y_list[i] = y;
    }


    return 0;
}