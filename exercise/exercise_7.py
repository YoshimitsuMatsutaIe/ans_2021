"""練習問題7：逆運動学（行列演算）
M1が作成したロボットアームの逆運動問題を解くプログラムを作成してください．
ロボットアームの構造はexercise_7_appendix.pdfに書いてあります．

ヤコビ行列を用いた逆運動学については以下のリンクで勉強してください．
リンク：http://www.thothchildren.com/chapter/5b4e1cb7103f2f3168711210

※exercise_7_utilsフォルダ内にあるモジュールを使っても良いです．
"""


import numpy as np
import scipy.integrate as integrate
import math
from math import pi, cos, sin, tanh


