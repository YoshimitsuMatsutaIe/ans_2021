### SciPyを使った実装 ###
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def diff_eq(x, t, a):
    """微分方程式"""
    return a * x

def do_example_2():
    time_list = np.linspace(0.0, 2.0, 100)  # 時間のリスト
    x_init = [1.0]  # 初期値
    
    a = 1
    
    # 解く
    sol = odeint(
        func = diff_eq,
        y0 = x_init,
        t = time_list,
        args = (a,)
    )  # scipy.integrate.odeintソルバー．他のソルバーもある．
    
    # グラフ化
    fig = plt.figure()  # figureインスタンスを作成
    ax = fig.add_subplot(111)  #figureオブジェクトにaxesを追加
    ax.plot(time_list, sol, label = "solution")  # プロットを入れる
    ax.set_xlabel('time')  # x軸にラベルを追加
    ax.set_ylabel('x')  # y軸にラベルを追加
    ax.grid(True)  # グリッドを入れる
    ax.legend()  # 凡例を入れる
    ax.set_aspect('equal', adjustable='box')  # 軸を揃える
    
    plt.show()  # プロットを表示


if __name__ == '__main__':
    do_example_2()