import math
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as amn


class PendulumAnim:
    """create pendulum animation
    
    これを参考にしました：
    https://qiita.com/trami/items/a1bdec427fc0420e6f19
    """
    
    def __init__(self):
        self.GRA_ACCEL = 9.8  # 重力加速度 [m/s^2]
        self.L = 1.0  # 振り子の紐の長さ [m]
        
        return
    
    
    def do(self):
        """execute"""
        
        self.run_sim()
        self.make_anim()
        
        return
    
    
    def run_sim(self):
        """振り子の時刻歴データを作成"""
        
        def derivs(t, state_vec):
            """振り子の運動方程式"""
            theta, omega = state_vec[0], state_vec[1]
            dthetadt = omega
            domegadt = -(self.GRA_ACCEL / self.L) * math.sin(theta)
            return [dthetadt, domegadt]
        
        self.time_span = [0, 20]                         # 観測時間 [s]
        self.dt = 0.05                               # 間隔 [s]
        self.t = np.arange(self.time_span[0], self.time_span[1], self.dt)
        theta_init = math.pi / 3  # 初期角度 [rad]
        omega_init = 0.0  # 初期角速度 [rad/s]
        state_vec_init = np.array([theta_init, omega_init])           # 初期状態
        
        # 解く
        self.sol = solve_ivp(
            fun = derivs,
            t_span = self.time_span,
            y0 = state_vec_init,
            t_eval = self.t,
        )
        
        return
    
    def make_anim(self):
        """アニメーション作成"""
        theta = self.sol.y[0,:]
        x = self.L * np.sin(theta)       # x = Lsin(theta)
        y = -self.L * np.cos(theta)      # y = -Lcos(theta)
        fig_ani, ax = plt.subplots()
        line, = ax.plot([], [], 'o-', linewidth=2) # このlineに次々と座標を代入して描画
        
        def onestep(i):
            """アニメーションのコールバック関数"""
            thisx = [0, x[i]]
            thisy = [0, y[i]]
            line.set_data(thisx, thisy)
            return line,
        
        ani = amn.FuncAnimation(
            fig = fig_ani, 
            func = onestep,
            frames = np.arange(0, len(self.t)),
            interval = 25,
            blit = True,
        )
        
        ax.set_xlim(-self.L*1.1, self.L*1.1)
        ax.set_ylim(-self.L*1.1, self.L*1.1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_aspect('equal')
        ax.grid()
        
        plt.show()
        return


if __name__ == '__main__':
    simu = PendulumAnim()
    simu.do()