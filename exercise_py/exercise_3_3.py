#!/usr/bin/env python

import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt
import matplotlib.animation as anm
from mpl_toolkits.mplot3d import Axes3D 

import time

class Particles:
    
    
    def __init__(self, **kwargs):
        
        self.N = kwargs.pop('N', 10)
        self.L = kwargs.pop('L', 1)
        self.DX_MAX = kwargs.pop('DX_MAX', 0.1)
        self.TIME_SPAN = kwargs.pop('TIME_SPAN', 10)
        self.TIME_INTERVAL = kwargs.pop('TIME_INTERVAL', 0.001)
        
        
        self.x = [(np.random.rand(3, 1) - 0.5)*self.L for i in range(self.N)]
        #print(self.x)
        self.dx = [(np.random.rand(3, 1)*2 - 1)*self.DX_MAX for i in range(self.N)]
    
        self.t = np.arange(0.0, self.TIME_SPAN, self.TIME_INTERVAL)
        self.time_list = list(self.t)
    
    
    def run_simu(self,):
        
        print('シミュレーション中...')
        start = time.time()
        
        n_x = np.array([[1, 0, 0]]).T
        n_y = np.array([[0, 1, 0]]).T
        n_z = np.array([[0, 0, 1]]).T
        
        self.t_x = np.eye(3) - 2 * n_x @ n_x.T
        self.t_y = np.eye(3) - 2 * n_y @ n_y.T
        self.t_z = np.eye(3) - 2 * n_z @ n_z.T
        
        
        def diff_eq(t, x):
            

            
            x_ = [np.array([x[i:i+3]]).T for i in range(self.N)]
            # dx_ = [
            #     -v if np.any(np.abs(x_[i]) > self.L/2) else v \
            #     for i, v in enumerate(self.dx)
            # ]
            
            dx_ = []
            for i, v in enumerate(self.dx):
                if abs(x_[i][0]) > self.L/2:
                    dx_.append(self.t_x @ v)
                elif abs(x_[i][1]) > self.L/2:
                    dx_.append(self.t_y @ v)
                elif abs(x_[i][2]) > self.L/2:
                    dx_.append(self.t_z @ v)
                else:
                    dx_.append(v)
            
            self.dx = dx_
            
            dx = [np.ravel(v).tolist() for v in dx_]
            return sum(dx, [])
        
        
        x_init_ = [np.ravel(x).tolist() for x in self.x]
        x_init = sum(x_init_, [])
        #print(x_init)
        sols = integrate.solve_ivp(
            fun = diff_eq,
            t_span = (0.0, self.TIME_SPAN),
            y0 = x_init,
            method = 'RK45',
            t_eval = self.t,
            args = None,
            #rtol = 1.e-12,
            #atol = 1.e-14,
        )
        
        self.sols = sols
        
        print('シミュレーション終了')
        print('処理時間 ', time.time() - start)
        return
    
    
    def draw(self,):
        
        fig_ani = plt.figure()
        ax = fig_ani.gca(projection = '3d')
        ax.grid(True)
        ax.set_xlabel('X[m]')
        ax.set_ylabel('Y[m]')
        ax.set_zlabel('Z[m]')
        ax.set_box_aspect((1,1,1))
        ax.set_xlim(-self.L/2, self.L/2)
        ax.set_ylim(-self.L/2, self.L/2)
        ax.set_zlim(-self.L/2, self.L/2)
        
        # for i in range(self.N):
        #     ax.plot(self.sols.y[i], self.sols.y[i+1], self.sols.y[i+2])
        
        ps = []
        for i in range(self.N):
            p_, = ax.plot([], [], [], marker='o')
            ps.append(p_)
        
        timeani = [ax.text(0.8, 0.2, 0.01, "time = 0.0 [s]", size = 10)]
        time_template = 'time = %s [s]'
        
        def update(i):
            
            for j, p in enumerate(ps):
                p.set_data(self.sols.y[j][i], self.sols.y[j+1][i],)
                p.set_3d_properties(self.sols.y[j+2][i])
            
            timeani.pop().remove()
            timeani_, = [ax.text(0.8, 0.12, 0.01, time_template % round(i*self.TIME_INTERVAL, 2), size = 10)]
            timeani.append(timeani_)
            
            return ps
        
        
        ani = anm.FuncAnimation(
            fig = fig_ani,
            func = update,
            frames = range(0, len(self.t)),
            interval = self.TIME_INTERVAL * 1.e-5,
        )
        
        ani.save('exercise_3_3.gif', writer='pillow')
        
        plt.show()
        
        return







if __name__ == '__main__':
    simu = Particles()
    simu.run_simu()
    simu.draw()