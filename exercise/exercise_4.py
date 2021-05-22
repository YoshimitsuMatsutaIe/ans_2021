"""練習問題4：PID制御（いろいろ）
1次元バネマスダンパ系を考えます．

1. 入力uを初期変位x0から変位xを目標位置xdに収束させるシミュレーションを行ってください．
入力uはPID制御で与えるものとします．
実行結果を横軸時間t，縦軸変位xのグラフで示して下さい．

2. 比例ゲイン，微分ゲイン，積分ゲインを変化させたとき，変位xの時間変化がどう変化するかアニメーションで示してください．
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anm


class PID:
    """"""
    
    def __init__(self, M = 1.0, K = 1.0, D = 1.0, INIT = 0.0, GOAL = 1.0):
        self.M = M  # 質量
        self.K = K  # ばね乗数
        self.D = D  # 摩擦係数
        self.INIT = INIT  # 初期値
        self.GOAL = GOAL  # 目標値
    
    
    def sim(self, Kp, Ki, Kd):
        """テスト"""
        
        
        
        u = 




if __name__ == '__main__':
    hoge = PID()
    