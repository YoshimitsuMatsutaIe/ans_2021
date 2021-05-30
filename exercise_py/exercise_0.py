"""練習問題0：素数判定（リスト，制御構文）
1. 与えられた任意の整数が素数か否かを判定する関数を作成してください．
2. 整数が入ったリストを与えたとき，リスト内に素数があるか否か判定する関数を作成してください．

ヒント：2では1で作った関数を利用すると良い
"""

from sympy import sieve


def main_1():
    """単純な実装"""
    def primality_test_1(n):
        """素数判定（単純な実装）"""
        n = int(n)
        print(n, 'is...')
        for i in range(2, n, 1):
            if n % i == 0:
                rezult = False
                break
            else:
                rezult = True
        print(rezult)
        return rezult

    def primality_test_2(n_list):
        rezult = []
        for n in n_list:
            rezult.append(primality_test_1(n))
        if True in rezult:
            print('There are prime numbers in the list!')
        else:
            print('There are no prime numbers in the list...')
    
    # 実行例
    primality_test_1(999961)  # 100万以下で最大の素数
    primality_test_1(1)
    primality_test_2([1, 6, 8, 108778979])
    
    return None


def main_2():
    """SciPyの素数判定モジュール？を仕様"""
    
    def primality_test_1(n):
        n = int(n)
        if n in sieve:
            print("True")
            rezult = True
        else:
            print('False')
            rezult =False
        return rezult

    def primality_test_2(n_list):
        rezult = []
        for n in n_list:
            rezult.append(primality_test_1(n))
        if True in rezult:
            print('There are prime numbers in the list!')
        else:
            print('There are no prime numbers in the list...')
    
    # 実行例
    primality_test_1(999961)  # 100万以下で最大の素数
    primality_test_1(1)
    primality_test_2([1, 6, 8, 108778979])
    
    return None



if __name__ == '__main__':
    main_1()
    #main_2()
