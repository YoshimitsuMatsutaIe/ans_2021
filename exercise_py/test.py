

# def diff_eq(a, b, func=lambda a, b: 0):
#     u = func(a, b)
#     return a + b + u

# def func_a(a, b):
#     return a * b

# a = 1
# b = 2

# c = diff_eq(a, b)
# print(c)



class Hoge:
    D = 123
    def __init__(self, a, B):
        self.a = a
        self.B = B
        print(self.D)
    

hoge = Hoge(2, 3)
print(hoge.a)
print(hoge.D)