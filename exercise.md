# exercise 

👇練習問題👇

## 練習問題0：素数判定（リスト，制御構文）
1. 与えられた任意の整数が素数か否かを判定する関数を作成してください．  
2. 整数が入ったリストを与えたとき，リスト内に素数があるか否か判定する関数を作成してください．  

~~ヒント：2では1で作った関数を利用すると良い~~  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_0.py)  
[matlab]()  



## 練習問題1：（ファイル操作，グラフ作成）
[/exercise_1_data](https://github.com/YoshimitsuMatsutaIe/ans_2021/tree/main/exercise_py/exercise_1_data)には，あるシステムの状態の時刻歴データが入っています．  
左から時刻t[sec]，位置x[m]，位置y[m]，...のデータです．  
フォルダ内のcsvファイルに対し，以下を実行するプログラムを作成してください．  

1. 位置の目標を(x,y)=(10, 0.5)とし，収束条件を次とします．  
収束条件：状態(x,y)が目標との誤差error_con_test[m]以内にtime_con_test[sec]以上留まる  
収束条件を与えたとき，全csvファイルに対しシステムが収束しているか判定し，収束したものをリストアップしてください．  

2. リストアップしたcsvファイルのx，yの時系列グラフを描画し，/exercise_1_rezultに保存する．  
ただしグラフのファイル名には実行時刻と使用したcsvファイル名が入るようにする．  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_1.py)  
[matlab]()  


## 練習問題2：van del Pol振動子（SciPy，制御構文，グラフ作成）
以下の状態方程式を解いてください．
数値積分の手法はなんでも良いです．  

<img src="https://latex.codecogs.com/png.latex?\bg_white&space;\frac{\mathrm{d}^2&space;x}{\mathrm{d}&space;t^2}&space;=&space;K(1-x^2)\frac{\mathrm{d}&space;x}{\mathrm{d}&space;t}-x">

また計算結果から次の三枚のグラフを作成してください．  
・横軸：時間，縦軸：位置  
・横軸：時間，縦軸：速度  
・横軸：位置，縦軸：速度  

ヒント：位置x_1と速度x_2を使って一階の微分方程式に直すと良い  


👇イメージ  
<img src="https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/misc/exercise_2.png" alt="ani" title="vandelpol">

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_2.py)  
[matlab]()  


## 練習問題3：アニメーション（グラフ作成）
どれか一つをやってください．  




### 1. 振り子
2次元平面で振り子が往復運動するアニメーションを作成してください．  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_3_1.py)  
[matlab]()  


### 2. 的あてゲーム
質点を斜方投射し移動目標にあてる的あてゲームを作成してください．  

要件  
・入力は質点の発射速度，発射角度のみ  
・移動目標はある区間を単振動で往復運動する  
・実行結果をアニメーションで示す  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_3_2.py)  
[matlab]()  


### 3. 3次元粒子
n個の粒子が立方体容器内を動き回るシミュレーションを行ってください．  

要件  
・粒子は壁で完全弾性衝突する  
・粒子同士の衝突は無視する  
・初期速度，初期位置はランダムとする  
・結果は3次元アニメーションで表示する  
・アニメーションの3軸スケールは揃える  

※余裕があれば粒子に半径を設けて，粒子間の衝突も実装して下さい．  

[python]()  
[matlab]()  


## 練習問題4：PID制御（いろいろ）
1次元バネマスダンパ系を考えます．  

<img src="https://latex.codecogs.com/gif.latex?\bg_white&space;m\frac{\mathrm{d}^2&space;x}{\mathrm{d}&space;t}&plus;c\frac{\mathrm{d}&space;x}{\mathrm{d}&space;t}&plus;kx=u"/>


1. 入力uを使って変位xを初期変位x0から目標位置xdに収束させるシミュレーションを行ってください．  
入力uはPID制御で与えるものとします．  
<img src="https://latex.codecogs.com/gif.latex?\bg_white&space;u=K_p(x_d-x)&plus;K_i\int_{0}^{t}(x_d-x)d\tau&plus;K_d\frac{\mathrm{d}&space;x}{\mathrm{d}&space;t}"/>
実行結果を横軸時間t，縦軸変位xのグラフで示して下さい．  

2. 比例ゲイン<img src="https://latex.codecogs.com/gif.latex?\bg_white&space;K_p">，積分ゲイン<img src="https://latex.codecogs.com/gif.latex?\bg_white&space;K_i">，微分ゲイン<img src="https://latex.codecogs.com/gif.latex?\bg_white&space;K_d">を変化させたとき，変位xの時間変化がどう変化するかアニメーションで示してください．  

👇イメージ  
<img src="https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/misc/exercise_4.gif" alt="ani" title="PID">  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_4.py)  
[matlab]()  


## 練習問題5：最適レギュレータ（いろいろ）
次の線形システムを考えます．  
<img src='https://latex.codecogs.com/gif.latex?\bg_white&space;\left\{\begin{matrix}&space;\dot{x}=&space;&&space;Ax&plus;Bu\\&space;y=&space;&&space;Cx&space;&plus;&space;Du&space;\end{matrix}\right.'/>  
リカッチ方程式を解いて最適な制御入力uを計算してください．  
またuを用いてモデルを制御した結果をグラフで示してください．  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_5.py)  
[matlab]()  


## 練習問題6：（クラス）
~~クラスを用いてexercise_2の内容を実装してください．~~  

~~ヒント：メソッドで微分方程式を解いたりグラフを作ったりできるVanDelPolクラスを作るとよい~~  
~~ヒント：exercise_2.pyモジュールをインポートしてファイル内の関数を使うと良い~~  

* 練習問題2と同じなので廃止


## 練習問題7：逆運動学（行列演算）
ロボットの逆運動問題を解くプログラムを作成してください．  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_7.py)  
[matlab]()  

## 練習問題8：倒立振り子（いろいろ）
~~実験室にある倒立振り子を制御するプログラムを作成してください．~~  
~~また実行結果をアニメーションで示してください．~~  
~~ただし使用する振子以下のものとします~~  
~~長さL[m]で重さM[kg]，密度は一定で厚さが無視できる棒~~  

倒立振り子を制御するプログラムを作成してください．

👇イメージ  
<img src="https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/misc/exercise_8__by_" alt="ani" title="pendulum">

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_8.py)  
[matlab]()  

## 練習問題9：最短経路探索（いろいろ）
任意のスタート位置からゴール位置までの最短経路を探索するプログラムを作成してください．  

[python](https://github.com/YoshimitsuMatsutaIe/ans_2021/blob/main/exercise_py/exercise_9.py)  
[matlab]()  

## 練習問題10：卒研に向けて
自分が選択した前期英語文献の再現実験を行ってください．  