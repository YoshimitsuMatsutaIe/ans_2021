### SciPyを使った実装 ###
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def diff_eq(x, t, a):
    """微分方程式"""
    return a * x

def do_example_2():
    time_list = np.arange(0.0, 2.0, 0.01)  # 時間のリスト
    x_init = [1.0]  # 初期値
    
    a = 1
    
    # 解く
    sol = solve_ivp(
        fun = diff_eq,
        y0 = x_init,
        t_span=(0.0, 2.0),
        t_eval = time_list,
        method = 'RK45',
        args = (a,)
    )  # scipy.integrate.odeintソルバー．他のソルバーもある．
    
    # グラフ化
    fig = plt.figure()  # figureインスタンスを作成
    ax = fig.add_subplot(111)  #figureオブジェクトにaxesを追加
    ax.plot(list(time_list), sol.y[0], label = "solution")  # プロットを入れる
    ax.set_xlabel('time')  # x軸にラベルを追加
    ax.set_ylabel('x')  # y軸にラベルを追加
    ax.grid(True)  # グリッドを入れる
    ax.legend()  # 凡例を入れる
    ax.set_aspect('equal', adjustable='box')  # 軸を揃える
    
    plt.show()  # プロットを表示


if __name__ == '__main__':
    do_example_2()