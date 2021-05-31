

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


import sympy as sy

x, dx, theta, dtheta, m, M, l, Dx, Dtheta, g= sy.symbols('x, dx, theta, dtheta, m, M, l, Dx, Dtheta, g')

multi = sy.Matrix([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, M + m, 0, m*l*sy.cos(theta)],
    [0, m*l*sy.cos(theta), 0, sy.Rational(4,3)*m*l**2],
])
offset_x = sy.Matrix([
    [dx],
    [dtheta],
    [m*l*dtheta**2*sy.sin(theta) - Dx * dx],
    [m*g*l*sy.sin(theta) - Dtheta + dtheta],
])
offset_u = sy.Matrix([[0, 0, 1, 0]]).T

Fa = multi.inv() * offset_x

print(Fa)