"""練習問題8：倒立振り子（いろいろ）
実験室にある倒立振り子を制御するプログラムを作成してください．
また実行結果をアニメーションで示してください．
ただし使用する振子以下のものとします
・長さL[m]で重さM[kg]，密度は一定で厚さが無視できる棒
"""

import numpy as np
import math
from math import pi, sin, cos, tan
import scipy.integrate as integrate
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import matplotlib.patches as patches
import time


class InvertedPendulum:
    """倒立振子のモデル"""
    
    def __init__(
        self,
        M_PEN=0.1, M_CAR=1.0, L=0.8, D_PEN=0.1, D_CAR=0.1, G_ACCEL=9.80665,
        TIME_INTERVAL=0.01, TIME_SPAN=5,
    ):
        
        self.M_PEN = M_PEN
        self.M_CAR = M_CAR
        self.L = L
        self.D_PEN = D_PEN
        self.D_CAR = D_CAR
        self.G_ACCEL = G_ACCEL
        self.TIME_INTERVAL = TIME_INTERVAL
        self.TIME_SPAN = TIME_SPAN
        
        self.t = np.arange(0.0, self.TIME_SPAN, self.TIME_INTERVAL)
        self.time_list = list(self.t)



class ByPID(InvertedPendulum):
    """PIDで制御"""
    
    def __init__(
        self, Kp=5.0, Ki=15.0, Kd=0.1,
        X_INIT=0.0, DX_INIT=0.0, THETA_INIT=pi/2, DTHETA_INIT=0.0, U_INIT = 0.0,
    ):
        """"""
        super().__init__()
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.THETA_INIT = THETA_INIT
        self.DTHETA_INIT = DTHETA_INIT
        self.U_INIT = U_INIT
        self.GOAL = pi/2
        
        self.do_simu()
    
    
    def diff_eq(self, t, x):
        """ODE"""
        
        multi = np.array([
            [1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, self.M_CAR + self.M_PEN, 0, 1/2*self.M_PEN*self.L*cos(x[2]), 0],
            [0, 1/2*self.M_PEN*self.L*cos(x[2]), 0, 1/3*self.M_PEN*(self.L**2), 0],
            [0, 0, 0, self.Kd, 1]
        ])
        offset = np.array([
            [x[1]],
            [x[3]],
            [x[4] + 1/2*self.M_PEN*self.L*sin(x[2])*(x[3]**2)],
            [1/2*self.M_PEN*self.L*sin(x[2])*x[1]*x[3] + 1/2*self.M_PEN*self.G_ACCEL*self.L*sin(x[2])],
            [-self.Kp*x[3] + self.Ki*(self.GOAL - x[2])],
        ])
        
        dx = np.linalg.inv(multi) @ offset
        
        return np.ravel(dx).tolist()
    
    
    def do_simu(self):
        """シミュレーションを実行"""
        
        state_init = [
            self.X_INIT, self.DX_INIT, self.THETA_INIT, self.DTHETA_INIT, self.U_INIT,
        ]
        
        # 解く
        sol = integrate.solve_ivp(
            fun = self.diff_eq,
            t_span = (0.0, self.TIME_SPAN),
            y0 = state_init,
            method = 'RK45',
            t_eval = self.t,
            args = None,
            rtol = 1.e-12,
            atol = 1.e-14,
        )
        
        # アニメーション化
        
        x_max = max(sol.y[0])
        x_min = min(sol.y[0])
        
        fig_ani = plt.figure()
        ax = fig_ani.add_subplot(111)
        ax.set_xlim(x_min-0.5, x_max+0.5)
        ax.set_xlim(-2, 2)
        #ax.set_ylim(-self.L*1.1, self.L*1.1)
        ax.set_aspect('equal')
        
        ax.plot([x_min, x_max], [0, 0], color = 'k')  # 水平線
        
        car = patches.Circle(
            xy = (self.X_INIT, 0),
            radius = self.L/10,
            ec = '#000000'
        )  # 台車
        ax.add_patch(car)
        
        pen, = ax.plot([], [], lw = 2)  # 振り子
        pen.set_data(
            [sol.y[0][0], sol.y[0][0] + self.L*cos(sol.y[2][0])],
            [0, self.L*sin(sol.y[2][0])],
        )
        
        def update(i):
            """アニメーションのコールバック"""
            
            car.set_center([sol.y[0][i], 0])
            
            pen.set_data(
                [sol.y[0][i], sol.y[0][i] + self.L*cos(sol.y[2][i])],
                [0, self.L*sin(sol.y[2][i])],
            )
            
            return [car, pen]
        
        ani = anm.FuncAnimation(
            fig = fig_ani,
            func = update,
            frames = np.arange(0, len(self.t)),
            interval = 25,
            blit = True,
        )
        
        # 状態変化のグラフ
        fig_state = plt.figure()
        ax_state = fig_state.add_subplot(111)
        ax_state.plot(self.time_list, sol.y[0])
        
        plt.show()



class LQR(InvertedPendulum):
    """リカッチで制御"""
    
    def __init__(self):
        super().__init__()


if __name__ == '__main__':
    sim_by_pid = ByPID()