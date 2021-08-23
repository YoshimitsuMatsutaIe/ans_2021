# include <iostream>
# include <fstream>


/**
 * @brief 素数判定関数．素直な実装．
 * @param[in]   n  調べたい正の整数
*/
bool isPrime(int n){
    if (n < 2){
        return false;
    }
    else if(n == 2){
        return true;
    }
    else {
        for (int i = 2; i < n; i++){
            if (n % i == 0){
                return false;
            }
            else {
                continue;
            }
        }
    }
    return true;
}


/**
 * @brief 2.を実行
 * @param[in]   n0  はじめ
 * @param[in]   n1  おわり
*/
void do_2(int n0, int n1){
    int count = 0;
    for (int i = n0; i < n1 + 1; i++){
        if (isPrime(i)){
            count += 1;
        }
    }

    std::cout << n0 << " から " << n1 << " に素数は " << std::endl;
    std::cout << count << std::endl;
    std::cout << "個ある" << std::endl;
}

int main(){

    int n0, n1;
    n0 = 1;
    n1 = 999961;
    do_2(n0, n1);

    return 0;
}