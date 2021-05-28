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

import exercise_3_1 as PendulumAnim

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
    hoge_1 = PendulumAnim()  # 振り子
    
    #hoge_2 = Matoate()  # 的あてゲーム