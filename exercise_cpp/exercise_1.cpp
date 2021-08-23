# include <iostream>
# include <fstream>
#include <cmath>
//using namespace std;


/**
 * @brief 素数判定関数
 * @param[in]   n  調べたい正の整数
*/
bool isPrime(int n){
    if (n < 2){
        return false;
    }
    else {
        for (int i = 2; i < n; i++){
            if (n % i == 0){
                return false;
            }
            else {
                continue;
            }
            return true;
        }
    }
}

int main(){
    int n;
    std::cout << "type number " << std::endl;
    std::cin >> n;
    std::cout << n << " is..." << std::endl;

    if (isPrime(n)){
        std::cout << "prime number!!" << std::endl;
    }
    else {
        std::cout << "orz" << std::endl;
    }
    return 0;
}