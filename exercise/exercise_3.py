"""練習問題3：アニメーション（グラフ作成）
どれか一つをやってください．


1. 振り子
2次元平面で振り子が往復運動するアニメーションを作成してください．

要件
・錘と設置点をつなぐ紐も描画してください．

ヒント：matplotlib.patchesを使用すると良い
-------------------------------------------------------------

2. 的あてゲーム
質点を斜方投射し移動目標にあてる的あてゲームを作成してください．

要件
・入力は質点の発射速度，発射角度のみ
・移動目標はある区間を単振動で往復運動する
・実行結果をアニメーションで示す

その他の仕様は自分で決めてください．
--------------------------------------------------------------

3. 3次元粒子
n個の粒子が立方体容器内を動き回るシミュレーションを行ってください．

要件
・粒子は壁で完全弾性衝突する
・粒子同士の衝突は無視する
・初期速度，初期位置はランダムとする
・結果は3次元アニメーションで表示する
・アニメーションの3軸スケールは揃える

※余裕があれば粒子に半径を設けて，粒子間の衝突も実装して下さい．
"""


import math
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as amn
import matplotlib.patches as patches


class PendulumAnim():
    """振り子\n
    これを参考にしました：
    https://qiita.com/trami/items/a1bdec427fc0420e6f19
    """
    
    def __init__(self):
        self.GRA_ACCEL = 9.8  # 重力加速度 [m/s^2]
        self.L = 1.0  # 振り子の紐の長さ [m]
        self.run_sim()
        self.make_anim()
    
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
            blit = True
        )
        
        ax.set_xlim(-self.L*1.1, self.L*1.1)
        ax.set_ylim(-self.L*1.1, self.L*1.1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_aspect('equal')
        ax.grid()
        
        plt.show()


class Matoate:
    """的あてゲーム\n
    
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




if __name__ == '__main__':
    #hoge_1 = PendulumAnim()  # 振り子
    
    hoge_2 = Matoate()  # 的あてゲーム