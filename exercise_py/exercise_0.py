#!/usr/bin/env python

from sympy import sieve
import time


def primality_test_simple(n):
    """素数判定（単純な実装）"""
    
    n = int(n)
    if n < 2:
        return False
    else:
        for i in range(2, n, 1):
            if n % i == 0:
                return False
    return True

def do_2(start, end, test_func):
    """素直な実装"""
    
    t1 = time.time()
    start = int(start)
    end = int(end)
    prime_list = []
    print("start ...")
    
    for n in range(start, end+1, 1):
        if test_func(n):
            prime_list.append(n)

    print('prime number is ...')
    #print(prime_list)
    print("Total = ", len(prime_list))
    print("time = ", time.time() - t1)
    return prime_list


def primality_test_using_sieve(n):
    """素数判定（sieve使用）
    
    本来の使い方じゃないので遅いかも
    """
    
    n = int(n)
    if n in sieve:
        return True
    else:
        return False



if __name__ == '__main__':

    #print(primality_test_simple(999961))  # 100万以下で最大の素数
    do_2(2, 34678, primality_test_using_sieve)

