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



def odeint_main():
    """SciPyのodeintを使った実装"""
    
    def func_van_del_pol(state_variable_vector, t, K):
        """van del Pol方程式
        """
        x_1 = state_variable_vector[0]
        x_2 = state_variable_vector[1]
        
        dx_1dt = x_2
        dx_2dt = K * (1-x_1**2) * x_2 - x_1
        
        return [dx_1dt, dx_2dt]

    time_list = np.linspace(0.0, 50.0, 1000000)
    state_variable_vector_init = [0.1, 0.1]
    K = 1

    #解く
    sol = integrate.odeint(
        func = func_van_del_pol,
        y0 = state_variable_vector_init,
        t = time_list,
        args = (K,)
    )

    # グラフ化
    label_name = ['x_1', 'x_2', 'trajetory',]
    xlabel_name = ['time', 'time', 'position x_1',]
    ylabel_name = ['position', 'velocity', 'velocity',]

    fig = plt.figure()
    axs = [fig.add_subplot(1, 3, i) for i in [1, 2, 3]]

    for i, ax in enumerate(axs):
        if i < 2:
            ax.plot(time_list, sol[:, i], label = label_name[i])
        else:
            ax.plot(sol[:, 0], sol[:, 1], label = label_name[i])
        ax.set_xlabel(xlabel_name[i])
        ax.set_ylabel(ylabel_name[i])
        ax.grid(True)
        ax.set_aspect('equal', adjustable='box')

    plt.show()
    
    return None


def solve_ivp_main():
    """SciPyのsolve_ivpを使った実装"""
    
    def func_van_del_pol(t, state_variable_vector, K):
        """van del Pol方程式
        """
        x_1 = state_variable_vector[0]
        x_2 = state_variable_vector[1]
        
        dx_1dt = x_2
        dx_2dt = K * (1-x_1**2) * x_2 - x_1
        
        return [dx_1dt, dx_2dt]

    t = np.arange(0.0, 50.0, 0.001)
    time_list = list(t)
    state_variable_vector_init = [0.1, 0.1]
    K = 1

    #解く
    sol = integrate.solve_ivp(
        fun = func_van_del_pol,
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
    ax.legend()
    plt.show()
    
    return None


if __name__ == '__main__':
    odeint_main()  # odeintは古いようです．
    #solve_ivp_main()
