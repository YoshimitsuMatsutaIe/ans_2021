"""練習問題2：van del Pol振動子（SciPy，制御構文，グラフ作成）
以下の状態方程式を解いてください．※Van der Pol振動子と呼ばれる有名な微分方程式です
数値積分の手法はなんでも良いです．

ddxddt = K * (1 - x**2) * dxdt - x

また計算結果から次の三枚のグラフを作成してください．
・横軸：時間，縦軸：位置
・横軸：時間，縦軸：速度
・横軸：位置，縦軸：速度

ヒント：位置x_1と速度x_2を使って一階の微分方程式に直すと良い
"""

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt


class VanDelPol:
    """VanDelPolを解く"""
    
    def __init__(
        self,
        K=1.0, x_init=0.1, dx_init=0.1, time_interval = 0.001, time_span = 50.0,
        solver = 'solve_ivp'
    ):
        """
        Parameters
        ---
        K : float
            係数
        x_init : float
            initial position
        dx_init : float
            initial velocity
        time_interval : float
            time interval
        time_span: float
            time span
        solver : str
            type of solver
        """
        
        self.K = K
        self.x_init = x_init
        self.dx_init = dx_init
        self.time_interval = time_interval
        self.time_span = time_span
        self.solver = solver
        
        self.t = np.arange(0.0, self.time_span, self.time_interval)
        self.time_list = list(self.t)
        
        data = self.run_simu()
        self.draw(data)
    
    
    def diff_eq(self, t, state, K):
        """ODE"""
        
        x_1, x_2 = state[0], state[1]
        
        dx_1dt = x_2
        dx_2dt = K * (1-x_1**2) * x_2 - x_1
        
        return [dx_1dt, dx_2dt]
    
    def run_simu(self):
        """run the simulation
        
        Return
        ---
        out : list
            [x_history, dx_history]
        """
        
        state_init = [self.x_init, self.dx_init]
        if self.solver == 'solve_ivp':
            sol = integrate.solve_ivp(
                fun = self.diff_eq,
                t_span = (0.0, self.time_span),
                y0 = state_init,
                method = 'RK45',
                t_eval = self.t,
                args = (self.K,),
                rtol = 1.e-12,  # 相対誤差
                atol = 1.e-14,  # 絶対誤差
            )
            return [sol.y[0], sol.y[1]]
        
        elif self.solver == 'odeint':
            sol = integrate.odeint(
                func = self.diff_eq,
                y0 = state_init,
                t = np.array(self.t),
                args = (self.K,),
                tfirst = True,
            )
            
            return [sol[:, 0], sol[:, 1]]
    
    
    def draw(self, data):
        """make figure
        
        Parameters
        ---
        data : list
            list of x, dx
        """
        
        label_name = ['x_1', 'x_2', 'trajetory',]
        xlabel_name = ['time', 'time', 'position x_1',]
        ylabel_name = ['position', 'velocity', 'velocity',]

        fig = plt.figure()
        axs = [fig.add_subplot(1, 3, i) for i in [1, 2, 3]]

        for i, ax in enumerate(axs):
            if i < 2:
                ax.plot(self.time_list, data[i], label = label_name[i])
            else:
                ax.plot(data[0], data[1], label = label_name[i])
            ax.set_xlabel(xlabel_name[i])
            ax.set_ylabel(ylabel_name[i])
            ax.grid(True)
            
            #ax.set_aspect('equal', adjustable='box')
        ax.legend()
        plt.show()


if __name__ == '__main__':
    simu = VanDelPol(solver='odeint')
