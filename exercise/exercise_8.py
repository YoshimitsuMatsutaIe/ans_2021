"""練習問題8：倒立振り子（いろいろ）
実験室にある倒立振り子を制御するプログラムを作成してください．
また実行結果をアニメーションで示してください．
ただし使用する振子以下のものとします
・長さL[m]で重さM[kg]，密度は一定で厚さが無視できる棒
"""

import numpy as np
import math
from math import pi, sin, cos, tan
import scipy as sp
import scipy.integrate as integrate
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import matplotlib.patches as patches
import time

import exercise_5

class InvertedPendulum:
    """倒立振子のモデル"""
    
    def __init__(
        self,
        M_PEN=1.0, M_CAR=5.0, L=1.5, D_PEN=0.01, D_CAR=0.01,
        TIME_INTERVAL=0.05, TIME_SPAN=10,
    ):
        """
        Parameters
        ---
        M_PEN :
            振り子の質量[kg]
        M_CAR :
            台車の質量[kg]
        L :
            振り子の長さ[m]
        D_PEN :
            抵抗[]
        D_CAR :
            動摩擦[]
        G_ACCEL :
            重力加速度[m/s^2]
        TIME_INTERVAL :
            刻み時間[sec]
        TIME_SPAN :
            シミュレーション時間[sec]
        """
        
        self.M_PEN = M_PEN
        self.M_CAR = M_CAR
        self.L = L
        self.D_PEN = D_PEN
        self.D_CAR = D_CAR
        self.G_ACCEL = 9.80665
        self.TIME_INTERVAL = TIME_INTERVAL
        self.TIME_SPAN = TIME_SPAN
        
        # 原点で線形化するとき
        offset_x = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 1],
            [0, -self.D_CAR, 0, 0],
            [0, 0, self.M_PEN*self.G_ACCEL*self.L, -self.D_PEN],
        ])
        offset_u = np.array([[0, 0, 1, 0]]).T
        multi = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, self.M_CAR + self.M_PEN, 0, self.M_PEN*self.L],
            [0, self.M_PEN*self.L, 0, 4/3*self.M_PEN*self.L**2],
        ])
        
        self.A_linier = np.linalg.inv(multi) @ offset_x
        self.B_linier = np.linalg.inv(multi) @ offset_u
        
        # 時間関係
        self.t = np.arange(0.0, self.TIME_SPAN, self.TIME_INTERVAL)
        self.time_list = list(self.t)
    
    
    def draw(self, x_list, theta_list, dx_list=None, dtheta_list=None, u_list=None):
        """結果を図示"""
        
        # アニメーション化
        x_max = max(x_list)
        x_min = min(x_list)
        
        fig_ani = plt.figure()
        ax = fig_ani.add_subplot(111)
        ax.set_xlim(x_min-0.5, x_max+0.5)
        ax.set_ylim(-self.L*1.2, self.L*1.2)
        ax.set_aspect('equal')
        ax.grid(True)
        
        ax.plot([x_min-0.5, x_max+0.5], [0, 0], color = 'k')  # 水平線
        
        car = patches.Circle(
            xy = (self.X_INIT, 0),
            radius = self.L/10,
            ec = '#000000'
        )  # 台車
        ax.add_patch(car)
        
        pen, = ax.plot([], [], lw = 2)  # 振り子
        pen.set_data(
            [x_list[0], x_list[0] + self.L*cos(-pi/2-theta_list[0])],
            [0, self.L*sin(pi/2-theta_list[0])],
        )
        
        def update(i):
            """アニメーションのコールバック"""
            
            car.set_center([x_list[i], 0])
            pen.set_data(
                [x_list[i], x_list[i] + self.L*cos(-pi/2-theta_list[i])],
                [0, self.L*sin(pi/2-theta_list[i])],
            )
            
            return [car, pen]
        
        
        ani = anm.FuncAnimation(
            fig = fig_ani,
            func = update,
            frames = np.arange(0, len(self.t)),
            interval = 25,
            blit = True,
        )
        
        #ani.save('sxercise_8_PID.gif', writer='pillow')
        
        # 状態変化のグラフ
        data_list = [x_list, theta_list, dx_list, dtheta_list, u_list]
        labels = ('x', 'theta', 'dx', 'dtheta', 'u')
        units = (' [m]', ' [rad]', ' [m/s]', ' [rad/s]', ' [?]')
        for i, data in enumerate(data_list):
            if data is None:
                pass
            else:
                fig = plt.figure()
                ax = fig.add_subplot(111)
                ax.plot(self.time_list, data, label = labels[i])
                if labels[i] == 'theta':
                    ax.plot(self.time_list, [self.GOAL]*len(self.time_list), label = 'goal')
                ax.set_xlabel('time [sec]')
                ax.set_ylabel(labels[i] + units[i])
            ax.legend()
            ax.grid(True)
        
        plt.show()
        
        return



class ByPID(InvertedPendulum):
    """PIDで制御
    ・未完成
    """
    
    def __init__(
        self, Kp=1, Ki=10, Kd=3,
        X_INIT=0.0, DX_INIT=0.0, THETA_INIT=0.0, DTHETA_INIT=0.0,
    ):
        """
        Parameters
        ---
        Kp :
            比例ゲイン
        Ki :
            積分ゲイン
        Kd :
            微分ゲイン
        
        """
        super().__init__()
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.THETA_INIT = THETA_INIT
        self.DTHETA_INIT = DTHETA_INIT
        self.GOAL = 0
        self.U_INIT = Kp*(self.GOAL - THETA_INIT) - Kd*DX_INIT
        
        # 実行
        sol = self.do_simu()
        self.draw(x_list=sol.y[0], theta_list=sol.y[2])
    
    
    def do_simu(self):
        """シミュレーションを実行"""
        
        start = time.time()
        print('計算中...')
        
        def diff_eq(t, x):
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
                [1/2*self.M_PEN*self.L*sin(x[2])*(x[3]**2) + x[4]],
                [1/2*self.M_PEN*self.L*sin(x[2])*x[1]*x[3] + 1/2*self.M_PEN*self.G_ACCEL*self.L*sin(x[2])],
                [-self.Kp*x[3] + self.Ki*(self.GOAL - x[2])],
            ])
            
            dx = np.linalg.inv(multi) @ offset
            
            return np.ravel(dx).tolist()
            #return [dx[0,0], dx[1,0], dx[2,0], dx[3,0], 0]
        
        state_init = [
            self.X_INIT, self.DX_INIT, self.THETA_INIT, self.DTHETA_INIT, self.U_INIT,
        ]
        
        # 解く
        sol = integrate.solve_ivp(
            fun = diff_eq,
            t_span = (0.0, self.TIME_SPAN),
            y0 = state_init,
            method = 'RK45',
            t_eval = self.t,
            args = None,
            rtol = 1.e-12,
            atol = 1.e-14,
        )
        
        print('計算終了．処理時間=', time.time() - start)
        
        if max(sol.y[0]) > self.L*10 or min(sol.y[0]) < -self.L*10:
            print("発散")
        
        return sol


class ByStateFeedback(InvertedPendulum):
    """状態フィードバックで制御？"""
    
    def __init__(
        self, X_INIT=0.0, DX_INIT=0.0, THETA_INIT=0.0, DTHETA_INIT=0.0,
        K = np.array([[1, 1, 1, 1]])
    ):
        super().__init__()
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.THETA_INIT = THETA_INIT
        self.DTHETA_INIT = DTHETA_INIT
        self.GOAL = 0
        
        if exercise_5.ByLQR.controllability(self.A_linier, self.B_linier):
            print('可制御')
        else:
            print('可制御でない')
            return
        
        sol = self.do_simu(K)
        self.draw(x_list=sol.y[0], theta_list=sol.y[2])
    
    
    def do_simu(self, K):
        self.A_F = self.A_linier - self.B_linier @ K
        #print(self.A_F)
        def diff_eq(t, state):
            x = np.array([state]).T
            dx = self.A_F @ x
            return np.ravel(dx).tolist()
        
        state_init = [self.X_INIT, self.DX_INIT, self.THETA_INIT, self.DTHETA_INIT]
        
        sol = integrate.solve_ivp(
            fun = diff_eq,
            t_span = (0.0, self.TIME_SPAN),
            y0 = state_init,
            method = 'RK45',
            t_eval = self.t,
            args = None,
            rtol = 1.e-12,
            atol = 1.e-14,
        )
        
        return sol



class ByLQR(InvertedPendulum):
    """線形化してLQRで制御
    
    安定性が低い
    """
    
    def __init__(self, Q, R, X_INIT=0.0, DX_INIT=0.0, THETA_INIT=0.0, DTHETA_INIT=0.0,):
        super().__init__()
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.THETA_INIT = THETA_INIT
        self.DTHETA_INIT = DTHETA_INIT
        self.GOAL = 0
        
        if exercise_5.ByLQR.controllability(self.A_linier, self.B_linier):
            print('可制御')
        else:
            print('可制御でない')
            return
        
        sol = self.do_simu(Q, R)
        self.draw(x_list=sol.y[0], theta_list=sol.y[2])
    
    
    def do_simu(self, Q, R):
        
        P = sp.linalg.solve_continuous_are(
            a = self.A_linier,
            b = self.B_linier,
            q = Q,
            r = R,
        )  # ricatti_eqを解く
        
        A_bar = (self.A_linier - self.B_linier @ np.linalg.inv(R) @ self.B_linier.T @ P)
        
        def diff_eq(t, state, A_bar):
            x = np.array([state]).T
            dx = A_bar @ x
            return np.ravel(dx).tolist()
        
        state_init = [self.X_INIT, self.DX_INIT, self.THETA_INIT, self.DTHETA_INIT]
        
        sol = integrate.solve_ivp(
            fun = diff_eq,
            t_span = (0.0, self.TIME_SPAN),
            y0 = state_init,
            method = 'RK45',
            t_eval = self.t,
            args = (A_bar,),
            rtol = 1.e-12,
            atol = 1.e-14,
        )
        
        return sol




if __name__ == '__main__':
    #sim = ByPID(THETA_INIT=pi/10)
    
    #sim = ByStateFeedback(THETA_INIT=pi/10)
    
    Q = np.diag([10, 10, 100, 100])
    R = np.array([[1]]) * 0.01
    sim = ByLQR(Q, R, THETA_INIT = pi/6)