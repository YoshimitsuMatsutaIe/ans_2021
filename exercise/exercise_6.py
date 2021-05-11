"""練習問題6：（クラス）
クラスを用いてexercise_2の内容を実装してください．

ヒント：メソッドで微分方程式を解いたりグラフを作ったりできるVanDelPolクラスを作るとよい
ヒント：exercise_2.pyモジュールをインポートしてファイル内の関数を使うと良い
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class VanDelPol:
    """van del polのクラス\n
    ・あくまでも作成例です
    """
    
    def __init__(self,):
        pass
    
    def func_van_del_pol(self, t, state_variable_vector, K):
        """van del Pol方程式
        """
        x_1 = state_variable_vector[0]
        x_2 = state_variable_vector[1]
        
        dx_1dt = x_2
        dx_2dt = K * (1-x_1**2) * x_2 - x_1
        
        return [dx_1dt, dx_2dt]
    
    def do_exercise_2(self, K = 1):
        """exercise2の内容を実行"""
        
        t = np.arange(0.0, 50.0, 0.001)
        time_list = list(t)
        state_variable_vector_init = [0.1, 0.1]
        #解く
        sol = solve_ivp(
            fun = self.func_van_del_pol,
            t_span = (0.0, 50.0),
            y0 = state_variable_vector_init,
            method = 'RK45',
            t_eval = t,
            args = (K,),
            rtol = 1.e-12,  # 相対誤差
            atol = 1.e-14,  # 絶対誤差
        )
        
        # グラフ化
        label_name = ['x_1', 'x_2', 'trajetory',]
        xlabel_name = ['time', 'time', 'position x_1',]
        ylabel_name = ['position', 'velocity', 'velocity',]
        
        fig = plt.figure()
        axs = [fig.add_subplot(1, 3, i) for i in [1, 2, 3]]
        
        for i, ax in enumerate(axs):
            if i < 2:
                ax.plot(time_list, sol.y[i], label = label_name[i])
            else:
                ax.plot(sol.y[0], sol.y[1], label = label_name[i])
            ax.set_xlabel(xlabel_name[i])
            ax.set_ylabel(ylabel_name[i])
            ax.grid(True)
            ax.set_aspect('equal', adjustable='box')
        
        plt.show()


if __name__ == '__main__':
    wow = VanDelPol()
    wow.do_exercise_2()