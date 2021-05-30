"""例題3：アニメーション
点が等速で円軌道上を回るアニメーションを作成してください．
""" 

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as anm

def circle(r):
    theta = np.linspace(0, 2 * np.pi, 360*3)
    return r * [np.cos(theta), np.sin(theta)]

def example_3():
    r = 1
    data = circle(r)
    
    fig_ani = plt.figure()
    ax = fig_ani.add_subplot(111)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    
    c = patches.Circle(
        xy = (0, 0),
        radius = r,
        fill = False,
        ec = 'r',
        linewidth = 2,
        )
    ax.add_patch(c)  # axに円を追加
    
    point, = ax.plot([], [], 'o-', markersize=7)
    
    def update(i):
        """フレーム一枚を作成するコールバック関数
        """
        point.set_data(data[0][i], data[1][i])
        return [point]
    
    ani = anm.FuncAnimation(
        fig = fig_ani,  # figureオブジェクト
        func = update,  # コールバック関数
        frames = np.arange(0, 360*3, 1),  # コールバックを呼ぶ出す回数
        init_func = None,
        fargs = None,
        interval = 100 * 10e-3,  # 図が切り替わる時間間隔．[msec]
        blit = True,  # ？
    )  # アニメーションを作成
    
    plt.show()  # 図を表示


if __name__ == '__main__':
    example_3()
