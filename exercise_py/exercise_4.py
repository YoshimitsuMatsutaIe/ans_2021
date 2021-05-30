import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import time
# import control
# import control.matlab


class SpringMassDamperModel:
    """spring mass damper system
    
    Attributes:
    -----
    """
    
    def __init__(
        self,
        M = 1.0, K = 1.0, C = 1.0,
        X_INIT = 0.0, DX_INIT = 0.0,
        GOAL = 1.0, TIME_INTERVAL = 0.01, TIME_SPAN = 10,
    ):
        """
        Parameters
        ----
        M : float
            mass [kg]
        K : float
            spring constant [N/m]
        C : float
            coefficient
        X_INIT : float
            initial position
        DX_INIT : float
            initial velocity
        GOAL : float
            goal position
        TIME_INTERVAL : float
            time interval
        TiME_SPAN : float
            time span
        """
        
        self.M = M
        self.K = K
        self.C = C
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.GOAL = GOAL
        self.TIME_INTERVAL = TIME_INTERVAL
        self.TIME_SPAN = TIME_SPAN
        
        return
    
    
    def diff_eq(self, t, state, Kp, Ki, Kd):
        """ODE
        
        Parameters
        ---
        t : range
            ?
        state : list
            generalized coordinate vector
        Kp : float
            proportional gain. non-negative.
        Ki : float
            integral gain. non-negative.
        Kd : float
            derivative gain. non-negative.
        
        Returns
        ---
        out : list
            generalized velocity vector
        """
        
        multi = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, Kd, 1],
        ])
        offset = np.array([
            [state[1]],
            [(state[2] - self.C*state[1] - self.K*state[0]) / self.M],
            [-Kp*state[1] + Ki*(self.GOAL - state[0])],
        ])
        
        dx = np.linalg.inv(multi) @ offset
        return np.ravel(dx).tolist()
    
    
    def do_exercise_4(
        self,
        Kp_range = [5.0], Ki_range = [5.0], Kd_range = [0, 1.5],
        part_num = 30, save = False,
    ):
        """do all
        
        create animation that changes all gains at the same time.
        
        Parameters
        -----
        Kp_rnage : list
            比例ゲイン．開始と終了（長さ2のリスト），または固定値（長さ1のリスト）を入れる．
        Ki_range : list
            積分ゲイン．Kp_rangeと同様．
        Kd_range : list
            微分ゲイン．Kp_rangeと同様．
        part_num : int
            分割数．大きいほど滑らか．
        save : bool
        
        Returns
        ----
        out : None
        """
        
        if len(Kp_range) == 1:
            if len(Ki_range) == 1:
                if len(Kd_range) == 1:
                    print('アニメーション無し')
                    part_num = 1
        
        
        t = np.arange(0.0, self.TIME_SPAN, self.TIME_INTERVAL)
        time_list = list(t)
        
        gain_ranges = [Kp_range, Ki_range, Kd_range]
        gain_lists = [
            gain*part_num if len(gain) == 1 else list(np.linspace(gain[0], gain[1], part_num)) \
                for gain in gain_ranges
        ]  # 各ゲインの数列の入ったリスト
        
        
        ### 微分方程式を説いてデータを作成 ###
        print('計算中...')
        start = time.time()
        sol_xs = []
        for i in range(part_num):
            U_INIT = gain_lists[0][i]*(self.GOAL - self.X_INIT) - gain_lists[2][i]*self.DX_INIT
            state_init = [self.X_INIT, self.DX_INIT, U_INIT]
            
            sol = integrate.solve_ivp(
                fun = self.diff_eq,
                t_span = (0.0, self.TIME_SPAN),
                y0 = state_init,
                method = 'RK45',
                t_eval = t,
                args = (gain_lists[0][i], gain_lists[1][i], gain_lists[2][i]),
                #rtol = 1.e-12,
                #atol = 1.e-14,
            )
            sol_xs.append(sol.y[0])  # 位置xのデータのみ格納
        print('計算終了\n処理時間', time.time() - start, 's')
        
        
        ### create animation ###
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
        ax.set_ylim(y_min-0.5, y_max+0.5)
        
        ax.plot(time_list, [self.GOAL]*len(time_list), color = 'red', label = 'goal')
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
            [ax.text(*comment_posi, comment_templates[i] % (gain_lists[i][0]), size = 10)] \
                for i, comment_posi in enumerate(comment_posis)
        ]
        
        def update(i):
            """callback"""
            
            curve.set_data(time_list, sol_xs[i])
            
            for j in range(3):
                comments[j].pop().remove()  # 最後の要素を取得して（pop），fig_aniから削除（remove）
                com_, = [
                    ax.text(*comment_posis[j], comment_templates[j] % (round(gain_lists[j][i], 1)), size = 10)
                ]
                comments[j].append(com_)  # 最後に追加
            
            return [curve]
        
        ani = anm.FuncAnimation(
            fig = fig_ani,
            func = update,
            frames = range(part_num),
            interval = 100,
        )
        print('アニメーション作成終了\n処理時間', time.time() - start, 's')
        
        ax.legend(loc = 8)
        plt.show()
        
        # アニメーションを保存
        if save:
            ani.save('exercise_4.gif', writer = 'pillow')
        
        return


def main():
    model = SpringMassDamperModel()
    model.do_exercise_4()


if __name__ == '__main__':
    main()
