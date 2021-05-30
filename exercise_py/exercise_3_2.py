"""練習問題3-2"""

import math
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as amn

class Matoate:
    """matate
    
    未完成です．ごめんなさい．
    """
    
    def __init__(self):
        self.goal_center = np.array([[0, 10]]).T
        self.goal_A = 2
        self.goal_lange = 0.1
        self.goal_omega = 0.5
        
        self.run()
        
    
    def run(self):
        """ゲームを実行"""
        
        while True:
            print('\n')
            print('的あてゲームへようこそ')
            print('\n')
            print('発射角度[degree]を入力してください')
            self.input_angle = math.radians(float(input()))
            if self.input_angle < 0 or self.input_angle > 180:
                print('それは無理')
                continue
            print('発射速度[m/s]を入力してください')
            self.input_velo = float(input())
            if self.input_velo < 0:
                print('それは無理')
                continue
            print('結果は．．．')
            
            self.run_anim()
    
    def run_anim(self):
        """計算&描画"""
        
        g = 9.8  # 重力加速度
        x_init = 0
        y_init = 0
        vx_init = self.input_velo * math.cos(self.input_angle)
        vy_init = self.input_velo * math.sin(self.input_angle)
        
        dt = 0.1
        t_end = (vy_init + math.sqrt(vy_init**2 + 2*g*y_init)) / g
        
        t = np.arange(0, t_end, dt)
        t_list = list(t)
        
        def calc(t, x0, v0, a):
            return x0 + v0 * t + 1/2 * a * t**2
        
        x = calc(t_list, x_init, vx_init, 0)
        y = calc(t_list, y_init, vy_init, -g)
        
        fig = plt.fogure()
        ax = fig.add_subplot(111)
        ax.plot(x, y)
        plt.show()