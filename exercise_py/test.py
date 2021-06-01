""""テスト用
・無視して下さい
"""

# def diff_eq(a, b, func=lambda a, b: 0):
#     u = func(a, b)
#     return a + b + u

# def func_a(a, b):
#     return a * b

# a = 1
# b = 2

# c = diff_eq(a, b)
# print(c)



# class Hoge:
#     D = 123
#     def __init__(self, a, B):
#         self.a = a
#         self.B = B
#         print(self.D)


# hoge = Hoge(2, 3)
# print(hoge.a)
# print(hoge.D)


# import sympy as sy

# x, dx, theta, dtheta, m, M, l, Dx, Dtheta, g= sy.symbols('x, dx, theta, dtheta, m, M, l, Dx, Dtheta, g')

# multi = sy.Matrix([
#     [1, 0, 0, 0],
#     [0, 0, 1, 0],
#     [0, M + m, 0, m*l*sy.cos(theta)],
#     [0, m*l*sy.cos(theta), 0, sy.Rational(4,3)*m*l**2],
# ])
# offset_x = sy.Matrix([
#     [dx],
#     [dtheta],
#     [m*l*dtheta**2*sy.sin(theta) - Dx * dx],
#     [m*g*l*sy.sin(theta) - Dtheta + dtheta],
# ])
# offset_u = sy.Matrix([[0, 0, 1, 0]]).T

# Fa = multi.inv() * offset_x

# print(Fa)


# import numpy as np

# a = np.array([[1, 2], [3, 4]])
# b_list = [[a, a], [a, a]]

# b = np.block(b_list)
# print(b)

import numpy as np
import cvxpy as cvx

np.random.seed(1)
# 次元数
m = 3
n = 2
# 各種定数・変数
A = np.random.randn(m, n)
b = np.random.randn(m)
c = np.random.randn(n)
x = cvx.Variable(n) # 変数定義
# 問題設定
obj = cvx.Minimize(c.T @ x) # 最小化
constraint = [A @ x <= b] # 制約
prob = cvx.Problem(obj, constraint)# 問題
prob.solve(verbose=True) # 解く
# 表示
print("obj: ", prob.value)
print("x: ", x.value)