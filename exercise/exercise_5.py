"""練習問題5：最適レギュレータ（いろいろ）
リカッチ方程式を解いて最適な制御入力uを計算してください．
またuを用いてモデルを制御した結果をグラフで示してください．
"""

import numpy as np
import matplotlib.pyplot as plt
import control
import control.matlab
import scipy as sp
import scipy.integrate as integrate
import scipy.optimize as optimize


class Model:
    """状態方程式"""
    
    def __init__(self, A, B, C):
        self.A = A
        self.B = B
        self.C = C
    
    
    def draw(self, sol):
        """グラフ作成"""
        
        labels = ['x1', 'x2', 'x3']
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for i, label in enumerate(labels):
            ax.plot(self.t_list, sol.y[i], label = label)
        ax.set_xlim(self.t_range)
        ax.set_xlabel('time')
        ax.grid(True)
        ax.legend()
        plt.show()


class ByLQR(Model):
    """LQRで制御"""
    
    def __init__(self, A, B, C, Q, R, solver = 'scipy'):
        super().__init__(A, B, C)
        self.Q = Q
        self.R = R
        
        if self.controllability(self.A, self.B):
            print('可制御')
        else:
            print('可制御でない')
            return
        
        sol = self.run_simu(solver)
        self.draw(sol)
    
    
    def solve_ricatti(self, solver):
        """リカッチを解く"""
        
        if solver == 'control':
            """contorolライブラリを使用"""
            sys = control.ss(self.A, self.B, self.C, np.zeros([3, 1]))
            _, S, _ = control.lqr(sys.A, sys.B, self.Q, self.R)
            return S
        elif solver == 'scipy':
            """scipyを使用"""
            return sp.linalg.solve_continuous_are(
                a=self.A, b=self.B, r=self.R, q=self.Q
            )
        elif solver == 'arimoto_potter':
            """自作"""
            return self.solve_are_by_arimoto_potter(
                self.A, self.B, self.Q, self.R,
            )
    
    
    def run_simu(self, solver):
        """シミュレーションを実行"""
        
        def diff_eq(t, state, A_bar):
            """ODE"""
            x = np.array([state]).T
            dx = A_bar @ x
            return np.ravel(dx).tolist()
        
        self.t_range = (0.0, 5.0)
        
        P = self.solve_ricatti(solver)
        print('riccati_eqの解\n', P)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        A_bar = A - B @ K
        
        t = np.arange(self.t_range[0], self.t_range[1], 0.01)
        self.t_list = list(t)
        
        sol = integrate.solve_ivp(
            fun = diff_eq,
            t_span = self.t_range,
            y0 = [0.4, 0.5, 1.2],
            method = 'RK45',
            t_eval = t,
            args = (A_bar,),
        )
        
        return sol
    
    
    @classmethod
    def controllability(cls, A, B):
        """可制御性を判定"""
        
        U = B
        for n in range(A.shape[0]):
            U = np.concatenate([U, np.linalg.matrix_power(A, n) @ B], axis = 1)
        rank = np.linalg.matrix_rank(U)
        
        if rank == A.shape[0]:
            return True
        else:
            return False
    
    
    @classmethod
    def solve_are_by_arimoto_potter(cls, A, B, Q, R):
        """有本・ポッターの方法
        
        https://qiita.com/trgkpc/items/8210927d5b035912a153
        
        Return
        ---
        P : ndarray
            リッカチ方程式の解
        """
        n = A.shape[0]
        
        H = np.block([
            [A, -B @ np.linalg.inv(R) @ B.T],
            [-Q, -A.T],
        ])  # ハミルトン行列
        
        eigen_value, eigen_vector = np.linalg.eig(H)
        
        Y_, Z_ = [], []
        index_array = sorted(
            [i for i in range(2*n)],
            key = lambda x:eigen_value[x].real
        )
        for i in index_array[:n]:
            Y_.append(eigen_vector.T[i][:n])
            Z_.append(eigen_vector.T[i][n:])
        Y = np.array(Y_).T
        Z = np.array(Z_).T
        
        if np.linalg.det(Y) != 0:
            return Z @ np.linalg.inv(Y)
        else:
            print("Warning: Y is not regular matrix. Result may be wrong!")
            return Z @ np.linalg.pinv(Y)
    
    
    @classmethod
    def solve_are_by_o(cls, A, B, Q, R):
        """数値解法で求める"""
        
        n = A.shape[0]
        P_init = np.zeros((n, n))
        
        def diff_reccati(P, n, A, B, Q, R):
            """ricattiの非線形行列微分方程式"""
            P = np.array([P]).reshape((n, n))
            dP = -(P @ A) - (A.T @ P) + (P @ B @ np.linalg.inv(R) @ B.T @ P) - Q
            return np.ravel(dP).tolist()
        
        sol = optimize.fsolve(
            func = diff_reccati,
            x0 = P_init,
            args = (n, A, B, Q, R),
        )
        return np.array([sol]).reshape((n, n))


if __name__ == '__main__':
    # 状態方程式
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
    
    # 重み行列
    Q = np.diag([1, 1, 1]) * 1000
    R = np.array([[10]])
    
    #simu = ByLQR(A, B, C, Q, R, solver = 'scipy')
    
    sol = ByLQR.solve_are_by_o(A, B, Q, R)
    print(sol)
    sol2 = sp.linalg.solve_continuous_are(a = A, b = B, q = Q, r = R)
    print(sol2)