"""例題0：リスト操作，制御構文
1. 二次関数：
y = x**2 + x + 1
を考えます．xの値を0.1ずつ動かしたときの二次関数のとる値の入ったリストを作成してください．
"""

x_list = [round(i / 10, 1) for i in range(0, 100)]
y_list = [round((i**2 + i + 1), 1) for i in x_list]

print(y_list)