"""練習問題4：PID制御（いろいろ）
1次元バネマスダンパ系を考えます．

1. 入力uを初期変位x0から変位xを目標位置xdに収束させるシミュレーションを行ってください．
入力uはPID制御で与えるものとします．
実行結果を横軸時間t，縦軸変位xのグラフで示して下さい．

2. 比例ゲイン，微分ゲイン，積分ゲインを変化させたとき，変位xの時間変化がどう変化するかアニメーションで示してください．
"""

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import time
# import control
# import control.matlab


class MyPID:
    """自作PID
    
    controlモジュールを使わずにPIDを実行．
    
    Attributes:
    -----
    """
    
    def __init__(
        self,
        M = 1.0, K = 1.0, C = 1.0,
        X_INIT = 0.0, DX_INIT = 0.0, U_INIT = 0.0,
        GOAL = 1.0, TIME_INTERVAL = 0.01, TIME_SPAN = 10,
    ):
        """
        Parameters
        ----
        M :
            mass
        K :
            ばね定数
        C :
            減衰係数
        X_INIT :
            初期位置
        DX_INIT :
            初期速度
        U_INIT :
            初期入力
        GOAL :
            目標位置
        TIME_INTERVAL :
            刻み時間
        TiME_SPAN :
            シミュレーション時間
        """
        
        self.M = M
        self.K = K
        self.C = C
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.U_INIT = U_INIT
        self.GOAL = GOAL
        self.TIME_INTERVAL = TIME_INTERVAL
        self.TIME_SPAN = TIME_SPAN
    
    
    def diff_eq(self, t, state, Kp, Ki, Kd):
        """ODE
        
        integrate.solve_ivpに使うODE
        
        Args:
        ---
        t : range
            時間列
        state : list
            変数ベクトル
        Kp :
            比例ゲイン
        Ki :
            積分ゲイン
        Kd :
            微分ゲイン
        
        Returns:
        ---
        out :list
            変数ベクトル
        
        """
        
        x = np.array([[state[0], state[1], state[2]]]).T
        multi = np.array([
            [0, 1, 0],
            [-self.K/self.M, -self.C/self.M, 1/self.M],
            [-Ki + Kd*self.K/self.M, -Kp + Kd*self.C/self.M, -Kd/self.M],
        ])
        offset = np.array([[0, 0, Ki*self.GOAL]]).T
        
        dx = multi @ x + offset
        
        return [dx[0, 0], dx[1, 0], dx[2, 0]]
    
    
    def do_exercise_4(
        self,
        Kp_range = [5.0], Ki_range = [5.0], Kd_range = [1.5],
        part_num = 30, save = False,
    ):
        """言われたことををやる
        
        引数で与えたゲイン幅を等分し，全ゲインを同時に変化させるアニメーションを作成する
        
        Parameters:
        --------
        Kp_rnage : list
            比例ゲイン．開始と終了（長さ2のリスト），または固定値（長さ1のリスト）を入れる．
        Ki_range : list
            積分ゲイン．Kp_rangeと同様．
        Kd_range : list
            微分ゲイン．Kp_rangeと同様．
        part_num : int
            分割数．整数でないと多分エラー．大きいほど滑らか．
        save : bool
            保存するか否か．
        
        Returns:
        ----
        out : None
            何も返しません．
        """
        
        # (1)か(2)か判定
        if len(Kd_range) == 1 and len(Ki_range) == 1 and len(Kd_range) == 1:
            part_num = 1
        else:
            pass
        
        t = np.arange(0.0, self.TIME_SPAN, self.TIME_INTERVAL)
        time_list = list(t)
        state_init = [self.X_INIT, self.DX_INIT, self.U_INIT]
        
        gain_ranges = [Kp_range, Ki_range, Kd_range]
        gain_lists = [
            gain*part_num if len(gain) == 1 else list(np.linspace(gain[0], gain[1], part_num)) for gain in gain_ranges
        ]  # 各ゲインの数列の入ったリスト
        
        
        ### 微分方程式を説いてデータを作成 ###
        print('計算中...')
        start = time.time()
        sol_xs = []
        for i in range(part_num):
            sol = integrate.solve_ivp(
                fun = self.diff_eq,
                t_span = (0.0, self.TIME_SPAN),
                y0 = state_init,
                method = 'RK45',
                t_eval = t,
                args = (gain_lists[0][i], gain_lists[1][i], gain_lists[2][i]),
                rtol = 1.e-12,
                atol = 1.e-14,
            )
            sol_xs.append(sol.y[0])  # 位置xのデータのみ格納
        print('計算終了\n処理時間', time.time() - start, 's')
        
        
        ### アニメーション作成 ###
        print('アニメーション作成中...')
        start = time.time()
        
        fig_ani = plt.figure()
        ax = fig_ani.add_subplot(111)
        ax.grid(True)
        ax.set_xlabel('time')
        ax.set_ylabel('x')
        ax.set_xlim(0.0, self.TIME_SPAN)
        
        # 位置xの最小値と最大値を探す
        if self.X_INIT < self.GOAL:
            y_max = self.GOAL
            y_min = self.X_INIT
        else:
            y_max = self.X_INIT
            y_min = self.GOAL
        ax.set_ylim(y_min-0.2, y_max+0.2)
        
        ax.plot(time_list, [self.GOAL]*len(time_list), color = 'red', label = 'goal')  # 目標値のラインを引く
        curve, = ax.plot([], [], label = 'position')
        
        comment_posis = []
        for i in range(3):
            comment_posis.append(
                (
                    self.TIME_SPAN-1.5,
                    abs(self.GOAL-self.X_INIT)/2 + min(self.GOAL, self.X_INIT) - i*0.1 - 0.2,
                )
            )  # テキスト位置
        comment_templates = ['Kp = %s', 'Ki = %s', 'Kd = %s']
        comments = [
            [ax.text(*comment_posi, comment_templates[i] % (gain_lists[i][0]), size = 10)] for i, comment_posi in enumerate(comment_posis)
        ]
        
        def update(i):
            """アニメーションのコールバック"""
            
            curve.set_data(time_list, sol_xs[i])
            
            for j in range(3):
                comments[j].pop().remove()  # 最後の要素を取得して（pop），fig_aniから削除（remove）
                com_, = [ax.text(*comment_posis[j], comment_templates[j] % (round(gain_lists[j][i], 1)), size = 10)]
                comments[j].append(com_)  # 最後に追加
            
            return [curve]
        
        ani = anm.FuncAnimation(
            fig = fig_ani,
            func = update,
            frames = range(part_num),
            interval = 100,
            #blit = True,  # blitを使う場合はコールバック関数でreturnする必要がある
        )
        print('アニメーション作成終了\n処理時間', time.time() - start, 's')
        
        ax.legend(loc = 8)
        plt.show()
        
        if save:
            ani.save('exercise_4.gif', writer = 'pillow')
        
        return None


# class PIDbyControlModul:
#     """controモジュールを使った実装"""
    
#     def __init__(self, M = 1.0, K = 1.0, C = 1.0, GOAL = 1.0):
#         self.GOAL = GOAL
        
        
    
#     #def




if __name__ == '__main__':
    model = MyPID()
    model.do_exercise_4(
        Kp_range = [5.0],
        Ki_range = [0.0, 5.0],
        Kd_range = [0.0, 2],
        part_num = 30,
    )
    #model.do_exercise_4()