"""基礎
"""

### コメント ###
# シャープより後は無視される

# 複数行のコメント
"""
・ダブルクォーテーション3つで囲む
"""

'''
・シングルクォーテーション3つで囲んでも良い
'''



### 基本 ###
# 変数
a = 1  # 代入
b, c = 2, 3  # 同時に代入

# ターミナル表示
print("hello")  # print関数で表示できる
print("a = ", a, "\n")  #\nで改行


### 四則演算+alpha ###
a, b = 2, 3
print("a = ", a)
print("b = ", b)

# 四則演算
ans = a + b  # 加算
print("a + b = ", ans)

ans = a - b  # 減算
print("a - b = ", ans)

ans = a * b  # 乗算
print("a * b = ", ans)

ans = a / b  # 除算
print("a / b = ", ans)

ans = a // b  # 商
print("a // b = ", ans)

ans = a % b  # 余り
print("a % b = ", ans)

ans = a ** b  # 冪乗
print("a ** b = ", ans)

