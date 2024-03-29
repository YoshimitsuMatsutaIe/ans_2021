#!/usr/bin/env python

import numpy as np
import pathlib
import os
import datetime
import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def do_exercise_1(r = 1.0, time_span = 3.0, figure = False):
    """do all
    
    Parameters:
    ---
    r :
        tolerance radius used for convergence test[m]．
    time_span :
        convergence test time[sec]．
    figure : bool
        whether to create a graph
    """
    
    GOAL = np.array([[10, 0.5]]).T  # goal position
    
    cwd = os.path.dirname(__file__)
    path_data = pathlib.Path(cwd + r'/exercise_1_data').resolve()
    path_result = pathlib.Path(cwd + r'/exercise_1_result').resolve()
    os.makedirs(path_result, exist_ok=True)
    
    date_now = datetime.datetime.now()  # 日付取得．保存ファイル名用
    csv_names = path_data.glob('*.csv')  # csvファイルの名前を取得
    header = 'List of converged data. radius of convergence = ' + \
        str(r) + ', stable time = ' + str(time_span) + '.'
    result = [[header]]
    
    for j, n in enumerate(csv_names):
        file_name = n.name
        if file_name == 'hoge.csv':  # hoge.csvはstrなので除外
            continue
        path_csv = path_data / file_name
        data = np.loadtxt(path_csv, delimiter=',')
        
        time_interval = data[1, 0] - data[0, 0]  # time interval
        i_span = int(time_span / time_interval)  # time_spanを離散値に変換
        
        xy_last = data[data.shape[0]-i_span:, 1:3].T
        error = np.linalg.norm(GOAL - xy_last, axis = 0)
        
        if all(r - error > 0):
            result.append([file_name])
            temp = True
        else:
            temp = False
        
        if figure:
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.plot(data[:, 1], data[:, 2], label = 'trajectory')
            ax.scatter(
                GOAL[0, 0], GOAL[1, 0],
                marker = '*', color = 'red', label = 'goal'
            )
            c = patches.Circle(
                xy = tuple(GOAL),
                radius = r,
                fill = False,
                ec = 'r',
                linewidth = 2,
                )
            ax.add_patch(c)  # axに円を追加
            ax.grid(True)
            ax.set_aspect('equal')
            ax.legend()
            title = file_name
            if temp:
                title += ' convergence'
            else:
                title += ' divergence'
            ax.set_title(title)
    
    
    # 結果をcsvファイルで出力
    result_name = 'result_' + \
        date_now.strftime('%Y-%m-%d--%H-%M-%S') + \
            '.csv'
    path_result = path_result / result_name
    with open(path_result, 'x', newline="") as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerows(result)
    
    # 結果をターミナルに表示
    for s in result:
        print(s[0])
    
    plt.show()
    
    return


if __name__ == '__main__':
    do_exercise_1(figure = False)