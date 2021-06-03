# ans_2021


## Description
例題，練習問題とその解答例

* 例題 : <https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/example.md>
* 練習問題 : <https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise.md>

未完成の部分があります．

## Dependencies

* python 3.8.8


## Requirement

* numpy 1.19.2
* scipy 1.6.2
* matplotlib 3.3.4
* sympy 1.8
* pandas 1.2.4
* control 0.9.0


## Note

### 問題文の訂正

* 2021 June 3,  練習問題8の削除  
練習問題2をクラスを使って実装したため問題を削除します．  

* 2021 May 24,  練習問題7の訂正  
問題文中のexercise_7_appendix.pdfを作るのを忘れてました．  
ロボットアームに限らず好きなロボットの逆運動学問題を解いてみて下さい．  


* 2021 May 23,  練習問題1の訂正  
~~pathlibのアップデート？によりexample_1.pyが動作しなくなりました．~~  
example_1.pyにはバグがあります．  
パス関係の箇所を修正しておいてください．  


* 2021 May 10,  練習問題5の訂正  
exercise_5.pyに間違いがありました．  
プログラム中で与えたA, B, C：  
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
C = np.array([
    [1],
    [-2],
    [-8]
])  
は無視して，自分の好きな状態方程式を考えてください．  


## Licence

[MIT](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/LICENSE)

## Author

* Matsuta Yoshimitsu
* email : <sbb03104@edu.osakafu-u.ac.jp>
