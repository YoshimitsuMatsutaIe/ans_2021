"""練習問題5：最適レギュレータ（いろいろ）
リカッチ方程式を解いて最適な制御入力uを計算してください．
またuを用いてモデルを制御した結果をグラフで示してください．
"""
import numpy as np
import control
import control.matlab
import matplotlib.pyplot as plt

# 状態方程式のA, B, C
A = np.array([
    [1.1, 2.0, 3.0],
    [0, 0.95, 1.20],
    [1.2, 0.01, 10.5],
])
B = np.array([
    [1.0],
    [0.0],
    [0.847],
])
C = np.eye(3)


def main_control():
    """controlモジュールを利用"""
    
    sys = control.ss(A, B, C, np.zeros([3, 1]))  # 状態空間モデル
    
    # 重み行列
    Q = np.diag([10, 10, 10]) * 10
    R = 10
    
    K, _, _ = control.lqr(sys.A, sys.B, Q, R)  # 最適状態フィードバックゲインを計算
    K *= -1  # マイナスにする
    
    Acl = sys.A + sys.B * K
    sys_fb = control.ss(Acl, sys.B, sys.C, sys.D)
    
    td = np.arange(0, 5, 0.01)
    x0 = np.array([[0.4, 0.5, 1.2]]).T
    
    x, t = control.matlab.initial(sys_fb, td, x0)
    print(x.shape)
    
    
    # グラフ化
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(t, x[:, 0], label = 'x1')
    ax.plot(t, x[:, 1], label = 'x2')
    ax.plot(t, x[:, 2], label = 'x3')
    ax.set_xlabel('time')
    ax.legend()
    ax.grid()
    
    plt.show()


def main_jisaku():
    
    pass



if __name__ == '__main__':
    main_control()
    #main_jisaku()