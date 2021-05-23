"""練習問題1：（ファイル操作，グラフ作成）
exercise_1_dataフォルダには，あるシステムの状態の時刻歴データが入っています．
左から時刻t[sec]，位置x[m]，位置y[m]，...のデータです．
フォルダ内のcsvファイルに対し，以下を実行するプログラムを作成してください．

1. 位置の目標を(x,y)=(10, 0.5)とし，収束条件を次とします．
収束条件：状態(x,y)が目標との誤差error_con_test[m]以内にtime_con_test[sec]以上留まる
収束条件を与えたとき，全csvファイルに対しシステムが収束しているか判定し，収束したものをリストアップしてください．

2. リストアップしたcsvファイルのx，yの時系列グラフを描画し，exercise_1_rezultフォルダに保存する．
ただしグラフのファイル名には実行時刻と使用したcsvファイル名が入るようにする．
"""

import pandas as pd
import numpy as np
import pathlib
import os
import datetime
import csv

def do_exercise_1():
    """全部やる"""
    
    cwd = os.path.dirname(__file__)
    path_data = pathlib.Path(cwd + r'/exercise_1_data').resolve()  # exercize_1_dataフォルダの相対パス
    
    date_now = datetime.datetime.now()  # 日付取得．保存ファイル名用
    csv_names = path_data.glob('*.csv')  # csvファイルの名前を取得
    header = '収束しているデータ'
    result = [[header]]  # 結果格納
    
    for n in csv_names:
        file_name = n.name
        if file_name == 'hoge.csv':  # hoge.csvはstrなので除外．もっと頭の良い実装法があれば教えてください．
            continue
        print(file_name)
        path_csv = path_data / file_name
        data = np.loadtxt(path_csv, delimiter=',')
        print(data[0, :])
    
    
    return None


if __name__ == '__main__':
    do_exercise_1()