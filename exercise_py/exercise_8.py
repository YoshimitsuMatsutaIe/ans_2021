#!/usr/bin/env python

import numpy as np
#import math
from math import pi, sin, cos, tan
from numpy.lib.twodim_base import triu_indices
#from numpy.core.fromnumeric import size
import scipy as sp
import scipy.integrate as integrate
import scipy.optimize as optimize
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import matplotlib.patches as patches
#import time
#import cvxpy as cvx

#import exercise_5


class InvertedPendulum:
    """model of inverted pendulum"""
    
    G_ACCEL = 9.80665  # 
    GOAL = 0  # goal of theta
    TIME_INTERVAL = 0.1  #[sec]
    TIME_SPAN = 5  # [sec]
    
    def __init__(
        self,
        X_INIT, DX_INIT, THETA_INIT, DTHETA_INIT,
        M_PEN=1.0, M_CAR=5.0, L=1.5, D_PEN=0.01, D_CAR=0.01,
    ):
        """
        Parameters
        ---
        X_INIT : float
            initial position
        DX_INIT : float
            initial velocity
        THETA_INIT : float
            initial angle [rad]
        DTHETA_INIT : float
            initial angular velocity [rad/s]
        M_PEN : float
            mass of pendulum [kg]
        M_CAR : float
            mass of car [kg]
        L : float
            length of pendulum [m]
        D_PEN : float
            coefficient of friction []
        D_CAR : float
            coefficient of friction []
        """
        
        self.X_INIT = X_INIT
        self.DX_INIT = DX_INIT
        self.THETA_INIT = THETA_INIT
        self.DTHETA_INIT = DTHETA_INIT
        self.M_PEN = M_PEN
        self.M_CAR = M_CAR
        self.L = L
        self.HALF_L = L/2
        self.D_PEN = D_PEN
        self.D_CAR = D_CAR
        
        self.free = True
        self.method = 'free'
        
        self.integral_error = self.GOAL - THETA_INIT
        
        # linertheta=0, dtheta=0
        offset_x = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 1],
            [0, -self.D_CAR, 0, 0],
            [0, 0, self.M_PEN*self.G_ACCEL*self.HALF_L, -self.D_PEN],
        ])
        offset_u = np.array([[0, 0, 1, 0]]).T
        multi = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, self.M_CAR + self.M_PEN, 0, self.M_PEN*self.HALF_L],
            [0, self.M_PEN*self.HALF_L, 0, 4/3*self.M_PEN*self.HALF_L**2],
        ])
        
        self.A_linier = np.linalg.inv(multi) @ offset_x
        self.B_linier = np.linalg.inv(multi) @ offset_u
        
        self.t = np.arange(0.0, self.TIME_SPAN, self.TIME_INTERVAL)
        self.time_list = list(self.t)
        
        return
    
    
    def eom(self, t, state,):
        """eom of inverted pendulum
        
        function to pass to solve_ivp. compute the input using child class's method input.
        """
        
        if self.free:
            u = 0
        else:
            u = self.input(state)
        
        multi = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, self.M_CAR + self.M_PEN, 0, self.M_PEN*self.HALF_L*cos(state[2])],
            [0, self.M_PEN*self.HALF_L*cos(state[2]), 0, 4/3*self.M_PEN*self.HALF_L**2],
        ])
        offset_x = np.array([
            [state[1]],
            [state[3]],
            [self.M_PEN*self.HALF_L*(state[3]**2)*sin(state[2]) - self.D_CAR*state[1]],
            [self.M_PEN*self.G_ACCEL*self.HALF_L*sin(state[2]) - self.D_PEN*state[3]],
        ])
        offset_u = np.array([[0, 0, 1, 0]]).T
        
        dx = np.linalg.inv(multi) @ (offset_x + offset_u*u)
        return np.ravel(dx).tolist()
    
    
    def run_simu(self):
        """run simulation
        
        Return
        ---
        out : OdeResult
        """
        
        print('control by ' + self.method)
        state_init = [
            self.X_INIT, self.DX_INIT, self.THETA_INIT, self.DTHETA_INIT,
        ]
        
        sol = integrate.solve_ivp(
            fun = self.eom,
            t_span = (0.0, self.TIME_SPAN),
            y0 = state_init,
            method = 'RK45',
            t_eval = self.t,
            args = None,
            #rtol = 1.e-12,
            #atol = 1.e-14,
        )
        
        return sol
    
    
    def draw(
        self, x_list, theta_list, dx_list=None, dtheta_list=None, u_list=None,
        ani_save=False
    ):
        """create a graph
        
        Parameters
        ---
        ani_save : bool
            save animation.
        """
        
        fig_ani = plt.figure()
        ax_ani = fig_ani.add_subplot(111)
        ax_ani.set_xlim(-self.L*3, self.L*3)
        ax_ani.set_ylim(-self.L*1.2, self.L*1.2)
        ax_ani.set_xlabel('x [m]')
        ax_ani.set_ylabel('y [m]')
        ax_ani.set_aspect('equal')
        ax_ani.grid(True)
        
        ax_ani.plot([-self.L*3, self.L*3], [0, 0], color = 'k')  # support
        
        car_init = patches.Circle(
            xy = (self.X_INIT, 0),
            radius = self.L/10,
            fc = '#a9a9a9',
            ec = '#a9a9a9',
        )  # car
        ax_ani.add_patch(car_init)
        
        pen_init, = ax_ani.plot([], [], lw = 2, color = '#a9a9a9')  # pendulum
        pen_init.set_data(
            [x_list[0], x_list[0] + self.L*cos(pi/2-theta_list[0])],
            [0, self.L*sin(pi/2-theta_list[0])],
        )
        
        
        car = patches.Circle(
            xy = (self.X_INIT, 0),
            radius = self.L/10,
            ec = '#000000'
        )  # car
        ax_ani.add_patch(car)
        
        pen, = ax_ani.plot([], [], lw = 2)  # pendulum
        pen.set_data(
            [x_list[0], x_list[0] + self.L*cos(-pi/2-theta_list[0])],
            [0, self.L*sin(pi/2-theta_list[0])],
        )
        
        time_template = 'time = %s [sec]'
        time = [ax_ani.text(-0.5, -1, time_template % self.time_list[0], size = 10)]
        
        ax_ani.set_title('by ' + self.method)
        
        def update(i):
            """callback of animation"""
            
            car.set_center([x_list[i], 0])
            pen.set_data(
                [x_list[i], x_list[i] + self.L*cos(-pi/2-theta_list[i])],
                [0, self.L*sin(pi/2-theta_list[i])],
            )
            
            time.pop().remove()
            time_, = [
                ax_ani.text(-0.5, -1, time_template % (round(self.time_list[i], 1)), size = 10)
            ]
            time.append(time_)
            
            return [car, pen]
        
        
        ani = anm.FuncAnimation(
            fig = fig_ani,
            func = update,
            frames = np.arange(0, len(self.t)),
            interval = 25*4,
        )
        
        if ani_save:
            fig_ani_name = 'exercise_8__by_' + self.method + '.gif'
            ani.save(fig_ani_name, writer='pillow')
        
        # figure of state history
        data_list = [x_list, theta_list, dx_list, dtheta_list, u_list]
        labels = ('x', 'theta', 'dx', 'dtheta', 'u')
        units = (' [m]', ' [rad]', ' [m/s]', ' [rad/s]', ' [?]')
        for i, data in enumerate(data_list):
            if data is None:
                pass
            else:
                fig = plt.figure()
                ax = fig.add_subplot(111)
                ax.plot(self.time_list, data, label = labels[i])
                if labels[i] == 'theta':
                    ax.plot(self.time_list, [self.GOAL]*len(self.time_list), label = 'goal')
                ax.set_xlabel('time [sec]')
                ax.set_ylabel(labels[i] + units[i])
            ax.legend()
            ax.grid(True)
        
        plt.show()
        
        return


def main_no_input():
    """execute (no input)"""
    
    sim = InvertedPendulum(X_INIT=0.0, DX_INIT=0.0, THETA_INIT=pi/6, DTHETA_INIT=0.0,)
    sol = sim.run_simu()
    sim.draw(sol.y[0], sol.y[2])
    return


class ByPID(InvertedPendulum):
    """control by PID
    
    default parameter is is the lack of coordination.
    """
    
    def __init__(
        self, Kp=150, Ki=2, Kd=15,
        X_INIT=0.0, DX_INIT=0.0, THETA_INIT=pi/6, DTHETA_INIT=0.0,
    ):
        """
        Parameters
        ---
        Kp : float
            proportional gain. non-negative.
        Ki : float
            integral gain. non-negative.
        Kd : float
            derivative gain. non-negative.
        HOGE_INIT : float
            initial value
        """
        
        super().__init__(X_INIT, DX_INIT, THETA_INIT, DTHETA_INIT)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.free = False
        self.method = 'PID'
        
        return
    
    
    def do_exercise_8(self, ani_save=False,):
        """execute"""
        
        sol = self.run_simu()
        self.draw(x_list=sol.y[0], theta_list=sol.y[2], ani_save=ani_save)
        return
    
    
    def input(self, state):
        """compute input value
        
        Parameter
        ---
        state : list
            state vector
        
        Return
        ---
        out : float
            input value
        """
        
        u = -self.Kp*(self.GOAL - state[2]) + \
            self.Ki*(self.integral_error) + \
                self.Kd*(state[3])
        self.integral_error = self.integral_error - (self.GOAL - state[2])*self.TIME_INTERVAL
        
        return u



class ByLQR(InvertedPendulum):
    """control by LQR
    
    suppose anguler(theta) and anguler-velocity(dtheta) are sufficiently small
    """
    
    def __init__(
        self, Q=np.diag([10, 10, 100, 100]), R=np.array([[1]]) * 0.01,
        X_INIT=0.0, DX_INIT=0.0, THETA_INIT=pi/6, DTHETA_INIT=0.0,
    ):
        """
        Parameters
        ---
        Q : ndarray
            Weight matrix. (n, n)
        R : ndarray
            Weight matrix. (m, m)
        HOGE_INIT : float
            initial value
        """
        
        super().__init__(X_INIT, DX_INIT, THETA_INIT, DTHETA_INIT)
        self.Q = Q
        self.R = R
        
        self.free = False
        self.method = 'LQR'
        
        P = sp.linalg.solve_continuous_are(
            a = self.A_linier,
            b = self.B_linier,
            q = Q,
            r = R,
        )  # solve ricatti eq
        
        self.F = -np.linalg.inv(R) @ self.B_linier.T @ P  # optiomal feedback gain
        
        return
    
    
    def do_exercise_8(self, ani_save=False,):
        """execute"""
        
        sol = self.run_simu()
        self.draw(x_list=sol.y[0], theta_list=sol.y[2], ani_save=ani_save)
        return
    
    
    def input(self, state):
        """compute input value
        
        Parameter
        ---
        state : list
            state vector
        
        Return
        ---
        out : float
            input value
        """
        
        x = np.array([state]).T
        u = self.F @ x
        return u[0, 0]



class ByMPC(InvertedPendulum):
    """control by Model Predictive Control
    
    未完成
    """
    
    def __init__(
        self,
        q = np.array([
            [1000, 0, 0, 0],
            [0, 1000, 0, 0],
            [0, 0, 1000, 0],
            [0, 0, 0, 1000],
            ]),
        r = 0.001,
        time_horizon = 0.5,
        X_INIT=0.0, DX_INIT=0.0, THETA_INIT=pi/6, DTHETA_INIT=0.0,
    ):
        """
        Parameters
        ---
        Q : ndarray
            Weight matrix. (n, n)
        R : ndarray
            Weight matrix. (m, m)
        HOGE_INIT : float
            initial value
        """
        
        super().__init__(X_INIT, DX_INIT, THETA_INIT, DTHETA_INIT)
        self.n_horizon = int(time_horizon / self.TIME_INTERVAL)
        dim = self.A_linier.shape[0]  # dimension
        
        self.free = False
        self.method = 'MPC'
        
        # calculate coefficient matrix
        F_list = [np.linalg.matrix_power(self.A_linier, n) for n in range(1, self.n_horizon+1)]
        F = np.block(F_list).T * self.TIME_INTERVAL
        #print(self.F)
        
        G_list = []
        for i in range(1, self.n_horizon + 1):
            col_G = []
            for j in range(1, self.n_horizon + 1):
                if i - j < 0:
                    col_G.append(np.zeros((dim, 1)))
                elif i == j:
                    col_G.append(self.B_linier)
                else:
                    col_G.append(np.linalg.matrix_power(self.A_linier, i-j) @ self.B_linier)
            G_list.append(col_G)
        G = np.block(G_list) * self.TIME_INTERVAL
        #print(self.G)
        
        Q = np.tile(q, (self.n_horizon, self.n_horizon))
        #R = np.full((self.n_horizon, self.n_horizon), r)
        R = np.eye(self.n_horizon) * r
        
        self.multi_A = G.T @ Q @ G + R
        self.multi_B_coeff = 2 * F.T @ Q @ G
        
        #print(self.multi_A)
        #print(np.all(np.linalg.eigvals(self.multi_A) >= 0))
        
        # print(self.multi_A)
        # print(self.multi_B_coeff)
        
        self.input([X_INIT, DX_INIT, THETA_INIT, DTHETA_INIT])
        
        return
    
    
    def do_exercise_8(self, ani_save=False,):
        """execute"""
        
        sol = self.run_simu()
        self.draw(x_list=sol.y[0], theta_list=sol.y[2], ani_save=ani_save)
        return
    
    # def input(self, state):
    #     """compute input value
        
    #     Parameter
    #     ---
    #     state : list
    #         state vector
        
    #     Return
    #     ---
    #     out : float
    #         input value
    #     """
        
    #     x = np.array([state]).T
    #     x0 = x
        
    #     multi_B = x0.T @ self.multi_B_coeff
        
    #     U = cvx.Variable((self.n_horizon, 1))  # input matrix
    #     print(U.value)
        
    #     # problem
    #     obj = cvx.Minimize(U.T @ self.multi_A @ U + multi_B @ U)
    #     prob = cvx.Problem(obj)
    #     opt_U = prob.solve(verbose=True)
        
    #     print(opt_U)
        
    #     return 

    def input(self, state):
        """compute input value
        
        Parameter
        ---
        state : list
            state vector
        
        Return
        ---
        out : float
            input value
        """
        
        x = np.array([state]).T
        self.x0 = x
        
        self.multi_B = self.x0.T @ self.multi_B_coeff
        
        def obj(U_list):
            """object func"""
            U = np.array([U_list]).T
            J = U.T @ self.multi_A @ U + 2 * self.x0 @ self.multi_B @ U
            return J[0, 0]
        
        x0 = [0] * self.n_horizon
        # print(x0)
        opt_U_list = optimize.minimize(obj, x0, method="nelder-mead")
        #print(opt_U_list.x)
        
        if opt_U_list.x[0] > 1000:
            print("発散")
        print(opt_U_list.x[0])
        return opt_U_list.x[0]

if __name__ == '__main__':
    # simu = ByPID(THETA_INIT=pi/10,)
    # simu.do_exercise_8()
    
    
    simu = ByLQR()
    simu.do_exercise_8(ani_save=True)
    
    # main_no_input()
    
    # simu = ByMPC()
    # simu.do_exercise_8(ani_save=True)