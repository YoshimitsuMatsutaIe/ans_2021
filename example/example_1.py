"""例題1：ファイル操作
example_1_dataフォルダにあるcsvファイルの中で，変数xにある数値以上の値が含まれる物をリストアップするプログラムを作成してください．
結果はcsvファイルでexample_1_rezultフォルダに保存されるようにしてください．
ただし結果のファイル名には実行時刻が入るようにしてください．
"""

### Pandasを使用 ###
import pandas as pd
import pathlib
import datetime
import csv

def func_example_1(path_data, path_result, threshold):
    """処理をすべて行う関数"""
    
    date_now = datetime.datetime.now()  # 日付取得．保存ファイル名用
    csv_names = path_data.glob('*.csv')  # csvファイルの名前を取得
    header = 'Files with x of ' + str(threshold) + ' or more is'
    result = [[header]]  # 結果格納
    
    for n in csv_names:
        file_name = n.name
        path_csv = path_data / file_name
        df = pd.read_csv(path_csv)  # csvファイル読み込み
        df = df.fillna(0)  # 欠損値を0で置き換え
        if (df['x'] >= 40).any():
            result.append([file_name])
        else:
            pass
    
    # 結果をcsvファイルで出力
    result_name = 'rezult_' + date_now.strftime('%Y-%m-%d--%H-%M-%S') + '.csv'
    path_result = path_result / result_name
    with open(path_result, 'x', newline="") as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerows(result)
    
    return None


if __name__ == '__main__':
    re_path_data = pathlib.Path('example_1_data')  # exercize_1_dataフォルダの相対パス
    ab_path_data = re_path_data.resolve()  # 絶対パスに変換
    
    re_path_result = pathlib.Path('example_1_result')  # exercize_1_rezultフォルダの相対パス
    ab_path_result = re_path_result.resolve()  # 絶対パスに変換
    
    func_example_1(
        path_data = ab_path_data,
        path_result = ab_path_result,
        threshold = 40,
    )