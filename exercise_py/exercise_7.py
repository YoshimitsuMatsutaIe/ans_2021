#!/usr/bin/env python

import numpy as np
import scipy.optimize as optimize
from math import pi, cos, sin, tanh
import matplotlib.pyplot as plt


class Baxter:
    """baxter robot
    
    http://www.nihonbinary.co.jp/Products/Robot/baxter_reserch.html
    """
    
    L = 278e-3
    h = 64e-3
    H = 1104e-3
    L0 = 270.35e-3
    L1 = 69e-3
    L2 = 364.35e-3
    L3 = 69e-3
    L4 = 374.29e-3
    L5 = 10e-3
    L6 = 368.3e-3
    
    q_inist_position = np.array([[0, -31, 0, 43, 0, 42, 0]]).T * pi / 180
    
    q1_min, q1_max = -141, 51
    q2_min, q2_max = -123, 60
    q3_min, q3_max = -173, 173
    q4_min, q4_max = -3, 150
    q5_min, q5_max = -175, 175
    q6_min, q6_max = -90, 120
    q7_min, q7_max = -175, 175
    q_min = np.array([[q1_min, q2_min, q3_min, q4_min, q5_min, q6_min, q7_min]]).T * pi / 180
    q_max = np.array([[q1_max, q2_max, q3_max, q4_max, q5_max, q6_max, q7_max]]).T * pi / 180
    
    def origins(self, q):
        """世界座標系から見た局所座標系の原点座標をまとめて計算"""
        q1, q2, q3, q4, q5, q6, q7 = q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0], q[6, 0]
        o_Wo = np.array([
            [0],
            [0],
            [0]
            ])
        
        o_BL = np.array([
            [self.L],
            [-self.h],
            [self.H]
            ])
        
        o_0 = np.array([
            [self.L],
            [-self.h],
            [self.H + self.L0]
            ])
        
        o_1 = np.array([
            [self.L],
            [-self.h],
            [self.H + self.L0]
            ])
        
        o_2 = np.array([
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1)],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) - self.h],
            [self.H + self.L0]
            ])
        
        o_3 = np.array([
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2)],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) - cos(pi/4)*self.L2*cos(q1)*cos(q2) - self.h],
            [self.H + self.L0 - self.L2*sin(q2)]
            ])
        
        o_4 = np.array([
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) - cos(pi/4)*self.L2*cos(q1)*cos(q2) - cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) - self.h],
            [self.H + self.L0 - self.L2*sin(q2) - self.L3*cos(q2)*cos(q3)]
            ])
        
        o_5 = np.array([
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4))],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) - cos(pi/4)*self.L2*cos(q1)*cos(q2) - cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) + cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) - self.h],
            [self.H + self.L0 - self.L2*sin(q2) - self.L3*cos(q2)*cos(q3) - self.L4*(sin(q2)*cos(q4) + sin(q4)*cos(q2)*cos(q3))]
            ])
        
        o_6 = np.array([
            
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5))],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) - cos(pi/4)*self.L2*cos(q1)*cos(q2) - cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) + cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5)) - self.h],
            [self.H + self.L0 - self.L2*sin(q2) - self.L3*cos(q2)*cos(q3) - self.L4*(sin(q2)*cos(q4) + sin(q4)*cos(q2)*cos(q3)) + self.L5*((sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q2))]
            ])
        
        o_7 = np.array([
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5))],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) - cos(pi/4)*self.L2*cos(q1)*cos(q2) - cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) + cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5)) - self.h],
            [self.H + self.L0 - self.L2*sin(q2) - self.L3*cos(q2)*cos(q3) - self.L4*(sin(q2)*cos(q4) + sin(q4)*cos(q2)*cos(q3)) + self.L5*((sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q2))]
            ])

        o_GL = np.array([
            [self.L + cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5)) + self.L6*(cos(pi/4)*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5))*sin(q6) + cos(pi/4)*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5))*sin(q6) + cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) + cos(q1)*cos(q2)*cos(q4))*cos(q6) + cos(pi/4)*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*cos(q2)*cos(q4))*cos(q6))],
            [cos(pi/4)*self.L1*sin(q1) - cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) - cos(pi/4)*self.L2*cos(q1)*cos(q2) - cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1)) + cos(pi/4)*self.L4*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5)) + self.L6*(-cos(pi/4)*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5))*sin(q6) + cos(pi/4)*(((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5))*sin(q6) - cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) + cos(q1)*cos(q2)*cos(q4))*cos(q6) + cos(pi/4)*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*cos(q2)*cos(q4))*cos(q6)) - self.h],
            [self.H + self.L0 - self.L2*sin(q2) - self.L3*cos(q2)*cos(q3) - self.L4*(sin(q2)*cos(q4) + sin(q4)*cos(q2)*cos(q3)) + self.L5*((sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q2)) + self.L6*(((sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q2))*sin(q6) + (-sin(q2)*cos(q4) - sin(q4)*cos(q2)*cos(q3))*cos(q6))]
            ])
        
        return [o_Wo, o_BL, o_0, o_1, o_2, o_3, o_4, o_5, o_6, o_7, o_GL]
    
    # ヤコビ行列
    def jacobi_GL(self, q):
        q1, q2, q3, q4, q5, q6, q7 = q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0], q[6, 0]
        z = np.array([
            [-cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) - cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) + cos(pi/4)*self.L3*(sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1)) - cos(pi/4)*self.L4*((sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(pi/4)*self.L4*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*cos(q2)*cos(q4)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) + cos(pi/4)*self.L5*(((sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*cos(q5) + (-sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3))*sin(q5)) + self.L6*((cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5))*sin(q6) + (cos(pi/4)*((sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*cos(q5) + cos(pi/4)*(-sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3))*sin(q5))*sin(q6) + (cos(pi/4)*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) + cos(pi/4)*cos(q1)*cos(q2)*cos(q4))*cos(q6) + (cos(pi/4)*(sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*sin(q4) - cos(pi/4)*sin(q1)*cos(q2)*cos(q4))*cos(q6)), -cos(pi/4)*self.L2*sin(q1)*sin(q2) - cos(pi/4)*self.L2*sin(q2)*cos(q1) - cos(pi/4)*self.L3*sin(q1)*cos(q2)*cos(q3) - cos(pi/4)*self.L3*cos(q1)*cos(q2)*cos(q3) - cos(pi/4)*self.L4*(sin(q1)*sin(q2)*cos(q4) + sin(q1)*sin(q4)*cos(q2)*cos(q3)) - cos(pi/4)*self.L4*(sin(q2)*cos(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2)*cos(q3)) + cos(pi/4)*self.L5*((sin(q1)*sin(q2)*sin(q4) - sin(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q1)*sin(q3)*sin(q5)*cos(q2)) + cos(pi/4)*self.L5*((sin(q2)*sin(q4)*cos(q1) - cos(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q1)*cos(q2)) + self.L6*((cos(pi/4)*(sin(q1)*sin(q2)*sin(q4) - sin(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + cos(pi/4)*sin(q1)*sin(q3)*sin(q5)*cos(q2))*sin(q6) + (cos(pi/4)*(sin(q2)*sin(q4)*cos(q1) - cos(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + cos(pi/4)*sin(q3)*sin(q5)*cos(q1)*cos(q2))*sin(q6) + (-cos(pi/4)*sin(q1)*sin(q2)*cos(q4) - cos(pi/4)*sin(q1)*sin(q4)*cos(q2)*cos(q3))*cos(q6) + (-cos(pi/4)*sin(q2)*cos(q1)*cos(q4) - cos(pi/4)*sin(q4)*cos(q1)*cos(q2)*cos(q3))*cos(q6)), cos(pi/4)*self.L3*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1)) + cos(pi/4)*self.L3*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3)) - cos(pi/4)*self.L4*(sin(q1)*cos(q3) - sin(q2)*sin(q3)*cos(q1))*sin(q4) - cos(pi/4)*self.L4*(-sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3))*sin(q4) + cos(pi/4)*self.L5*((sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*sin(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4)*cos(q5)) + cos(pi/4)*self.L5*((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4)*cos(q5) + (sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*sin(q5)) + self.L6*((cos(pi/4)*(sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*sin(q5) + cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4)*cos(q5))*sin(q6) + cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q4)*cos(q6) + (cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4)*cos(q5) + cos(pi/4)*(sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*sin(q5))*sin(q6) + cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q4)*cos(q6)), -cos(pi/4)*self.L4*((sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*cos(q4) + sin(q4)*cos(q1)*cos(q2)) - cos(pi/4)*self.L4*((sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*cos(q4) + sin(q1)*sin(q4)*cos(q2)) + cos(pi/4)*self.L5*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4))*cos(q5) + cos(pi/4)*self.L5*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4))*cos(q5) + self.L6*(cos(pi/4)*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4))*sin(q6)*cos(q5) + (cos(pi/4)*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - cos(pi/4)*sin(q4)*cos(q1)*cos(q2))*cos(q6) + cos(pi/4)*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4))*sin(q6)*cos(q5) + (cos(pi/4)*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - cos(pi/4)*sin(q1)*sin(q4)*cos(q2))*cos(q6)), cos(pi/4)*self.L5*(-((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*sin(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q5)) + cos(pi/4)*self.L5*(-((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*sin(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q5)) + self.L6*((-cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*sin(q5) + cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q5))*sin(q6) + (-cos(pi/4)*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*sin(q5) + cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q5))*sin(q6)), self.L6*((cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5))*cos(q6) + (cos(pi/4)*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5))*cos(q6) - (cos(pi/4)*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) + cos(pi/4)*cos(q1)*cos(q2)*cos(q4))*sin(q6) - (cos(pi/4)*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + cos(pi/4)*sin(q1)*cos(q2)*cos(q4))*sin(q6)), 0],
            [cos(pi/4)*self.L1*sin(q1) + cos(pi/4)*self.L1*cos(q1) + cos(pi/4)*self.L2*sin(q1)*cos(q2) + cos(pi/4)*self.L2*cos(q1)*cos(q2) + cos(pi/4)*self.L3*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3)) - cos(pi/4)*self.L3*(sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1)) - cos(pi/4)*self.L4*((sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4)) + cos(pi/4)*self.L4*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + sin(q1)*cos(q2)*cos(q4)) + cos(pi/4)*self.L5*(((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5)) - cos(pi/4)*self.L5*(((sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*cos(q5) + (-sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3))*sin(q5)) + self.L6*((cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) + cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5))*sin(q6) + (-cos(pi/4)*((sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*cos(q5) - cos(pi/4)*(-sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3))*sin(q5))*sin(q6) + (cos(pi/4)*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) + cos(pi/4)*cos(q1)*cos(q2)*cos(q4))*cos(q6) + (-cos(pi/4)*(sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*sin(q4) + cos(pi/4)*sin(q1)*cos(q2)*cos(q4))*cos(q6)), -cos(pi/4)*self.L2*sin(q1)*sin(q2) + cos(pi/4)*self.L2*sin(q2)*cos(q1) - cos(pi/4)*self.L3*sin(q1)*cos(q2)*cos(q3) + cos(pi/4)*self.L3*cos(q1)*cos(q2)*cos(q3) - cos(pi/4)*self.L4*(sin(q1)*sin(q2)*cos(q4) + sin(q1)*sin(q4)*cos(q2)*cos(q3)) + cos(pi/4)*self.L4*(sin(q2)*cos(q1)*cos(q4) + sin(q4)*cos(q1)*cos(q2)*cos(q3)) + cos(pi/4)*self.L5*((sin(q1)*sin(q2)*sin(q4) - sin(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q1)*sin(q3)*sin(q5)*cos(q2)) - cos(pi/4)*self.L5*((sin(q2)*sin(q4)*cos(q1) - cos(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q1)*cos(q2)) + self.L6*((cos(pi/4)*(sin(q1)*sin(q2)*sin(q4) - sin(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) + cos(pi/4)*sin(q1)*sin(q3)*sin(q5)*cos(q2))*sin(q6) + (-cos(pi/4)*(sin(q2)*sin(q4)*cos(q1) - cos(q1)*cos(q2)*cos(q3)*cos(q4))*cos(q5) - cos(pi/4)*sin(q3)*sin(q5)*cos(q1)*cos(q2))*sin(q6) + (-cos(pi/4)*sin(q1)*sin(q2)*cos(q4) - cos(pi/4)*sin(q1)*sin(q4)*cos(q2)*cos(q3))*cos(q6) + (cos(pi/4)*sin(q2)*cos(q1)*cos(q4) + cos(pi/4)*sin(q4)*cos(q1)*cos(q2)*cos(q3))*cos(q6)), -cos(pi/4)*self.L3*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1)) + cos(pi/4)*self.L3*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3)) + cos(pi/4)*self.L4*(sin(q1)*cos(q3) - sin(q2)*sin(q3)*cos(q1))*sin(q4) - cos(pi/4)*self.L4*(-sin(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q3))*sin(q4) - cos(pi/4)*self.L5*((sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*sin(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4)*cos(q5)) + cos(pi/4)*self.L5*((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4)*cos(q5) + (sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*sin(q5)) + self.L6*((-cos(pi/4)*(sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*sin(q5) - cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4)*cos(q5))*sin(q6) - cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q4)*cos(q6) + (cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4)*cos(q5) + cos(pi/4)*(sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*sin(q5))*sin(q6) + cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q4)*cos(q6)), cos(pi/4)*self.L4*((sin(q1)*sin(q3) + sin(q2)*cos(q1)*cos(q3))*cos(q4) + sin(q4)*cos(q1)*cos(q2)) - cos(pi/4)*self.L4*((sin(q1)*sin(q2)*cos(q3) - sin(q3)*cos(q1))*cos(q4) + sin(q1)*sin(q4)*cos(q2)) - cos(pi/4)*self.L5*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4))*cos(q5) + cos(pi/4)*self.L5*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4))*cos(q5) + self.L6*(-cos(pi/4)*(-(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(q1)*cos(q2)*cos(q4))*sin(q6)*cos(q5) + (-cos(pi/4)*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) + cos(pi/4)*sin(q4)*cos(q1)*cos(q2))*cos(q6) + cos(pi/4)*(-(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) - sin(q1)*cos(q2)*cos(q4))*sin(q6)*cos(q5) + (cos(pi/4)*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - cos(pi/4)*sin(q1)*sin(q4)*cos(q2))*cos(q6)), -cos(pi/4)*self.L5*(-((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*sin(q5) + (-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q5)) + cos(pi/4)*self.L5*(-((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*sin(q5) + (sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q5)) + self.L6*((cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*sin(q5) - cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q5))*sin(q6) + (-cos(pi/4)*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*sin(q5) + cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q5))*sin(q6)), self.L6*((-cos(pi/4)*((-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q4) - sin(q4)*cos(q1)*cos(q2))*cos(q5) - cos(pi/4)*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q5))*cos(q6) + (cos(pi/4)*((-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q4) - sin(q1)*sin(q4)*cos(q2))*cos(q5) + cos(pi/4)*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q5))*cos(q6) - (-cos(pi/4)*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q4) - cos(pi/4)*cos(q1)*cos(q2)*cos(q4))*sin(q6) - (cos(pi/4)*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q4) + cos(pi/4)*sin(q1)*cos(q2)*cos(q4))*sin(q6)), 0],
            [0, -self.L2*cos(q2) + self.L3*sin(q2)*cos(q3) - self.L4*(-sin(q2)*sin(q4)*cos(q3) + cos(q2)*cos(q4)) + self.L5*((sin(q2)*cos(q3)*cos(q4) + sin(q4)*cos(q2))*cos(q5) - sin(q2)*sin(q3)*sin(q5)) + self.L6*(((sin(q2)*cos(q3)*cos(q4) + sin(q4)*cos(q2))*cos(q5) - sin(q2)*sin(q3)*sin(q5))*sin(q6) + (sin(q2)*sin(q4)*cos(q3) - cos(q2)*cos(q4))*cos(q6)), self.L3*sin(q3)*cos(q2) + self.L4*sin(q3)*sin(q4)*cos(q2) + self.L5*(sin(q3)*cos(q2)*cos(q4)*cos(q5) + sin(q5)*cos(q2)*cos(q3)) + self.L6*((sin(q3)*cos(q2)*cos(q4)*cos(q5) + sin(q5)*cos(q2)*cos(q3))*sin(q6) + sin(q3)*sin(q4)*cos(q2)*cos(q6)), -self.L4*(-sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + self.L5*(sin(q2)*cos(q4) + sin(q4)*cos(q2)*cos(q3))*cos(q5) + self.L6*((sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*cos(q6) + (sin(q2)*cos(q4) + sin(q4)*cos(q2)*cos(q3))*sin(q6)*cos(q5)), self.L5*(-(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*sin(q5) + sin(q3)*cos(q2)*cos(q5)) + self.L6*(-(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*sin(q5) + sin(q3)*cos(q2)*cos(q5))*sin(q6), self.L6*(((sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))*cos(q5) + sin(q3)*sin(q5)*cos(q2))*cos(q6) - (-sin(q2)*cos(q4) - sin(q4)*cos(q2)*cos(q3))*sin(q6)), 0]
            ])
        return z


class Kinematics(Baxter):
    """運動学を解く"""
    
    def forward(
        self,
        q = np.array([[0, -31, 0, 43, 0, 42, 0]]).T * pi / 180,
    ):
        """compute forward kinematics
        
        Parameters
        ---
        q : ndarray
            joint angle vector
        """
        
        return self.origins(q)
    
    
    def inverse(self, xd, gain=1, q_limit=False):
        """compute inverse kinematics
        
        Parameters
        ---
        xd : ndarray
            desired glipper position (vector)
        gain : float
            0 < gain
        q_limit : bool
            ジョイント角度の制限を考慮するか否か
        """
        
        def func(q, xd, g):
            q = np.array([q]).T
            x = self.forward(q)[10]
            J = self.jacobi_GL(q)
            dq = g * np.linalg.pinv(J) @ (xd - x)
            
            return np.ravel(dq).tolist()
        
        for n in range(10000):
            print('trial', n, '...')
            q_init = [np.random.rand()*2*pi for i in range(7)]
            
            qd = optimize.fsolve(
                func = func,
                x0 = q_init,
                args = (xd, gain),
            )
            qd = np.array([qd]).T
            
            if q_limit:
                if all(qd - self.q_min > 0) and all(self.q_max - qd > 0):
                    print('succes!')
                    break
            else:
                break
            
        error = np.linalg.norm(self.forward(qd)[10] - xd)
        print('error = ', error, '[m]')
        
        return qd
    
    
    def draw(self, q, xd=None):
        
        fig = plt.figure()
        ax = fig.gca(projection = '3d')
        ax.grid(True)
        ax.set_xlabel('X[m]')
        ax.set_ylabel('Y[m]')
        ax.set_zlabel('Z[m]')
        ax.set_box_aspect((1,1,1))
        
        joints_init_ = self.forward(self.q_inist_position)
        joints_init = joints_init_[0]
        for i in range(1, len(joints_init_)):
            joints_init = np.concatenate([joints_init, joints_init_[i]], axis = 1)
        ax.plot(
            joints_init[0, :], joints_init[1, :], joints_init[2, :],
            marker = 'o', label = 'initial configuration', color = '#808080',
        )
        
        joints_ = self.forward(q)
        joints = joints_[0]
        for i in range(1, len(joints_)):
            joints = np.concatenate([joints, joints_[i]], axis = 1)
        ax.plot(
            joints[0, :], joints[1, :], joints[2, :],
            marker = 'o', label = 'solution configuration'
        )
        
        
        if xd is not None:
            ax.scatter(
                xd[0, 0], xd[1, 0], xd[2, 0], 
                marker = '*', linewidths = 1.5, color = 'red',
                label = 'desired glipper position',
            )
        
        ax.legend()
        plt.show()
        
        return
    
    
    def do_exercise_7(self,xd):
        
        qd = self.inverse(xd)
        self.draw(qd, xd)
        
        return



if __name__ == '__main__':
    model = Kinematics()
    model.do_exercise_7(xd = np.array([[0.3, -0.6, 1]]).T)
