### Pandasを使用 ###
import pandas as pd
import pathlib
import os
import datetime
import csv

def func_example_1(threshold):
    """処理をすべて行う関数"""
    
    cwd = os.path.dirname(__file__)  # 実行しているpyファイルのカレントディレクトリ取得
    re_path_data = pathlib.Path(cwd + r'/example_1_data')  # exercize_1_dataフォルダの相対パス
    ab_path_data = re_path_data.resolve()  # 絶対パスに変換
    re_path_result = pathlib.Path(cwd + r'/example_1_result')  # exercize_1_rezultフォルダの相対パス
    ab_path_result = re_path_result.resolve()  # 絶対パスに変換
    os.makedirs(ab_path_result, exist_ok=True)  # exercize_1_resultフォルダが無い場合は作成
    
    date_now = datetime.datetime.now()  # 日付取得．保存ファイル名用
    
    csv_names = ab_path_data.glob('*.csv')  # csvファイルの名前を取得
    
    header = 'Files with x of ' + str(threshold) + ' or more is'
    result = [[header]]  # 結果格納
    
    for n in csv_names:
        file_name = n.name
        path_csv = ab_path_data / file_name
        df = pd.read_csv(path_csv)  # csvファイル読み込み
        df = df.fillna(0)  # 欠損値を0で置き換え
        if (df['x'] >= threshold).any():
            result.append([file_name])
        else:
            pass
    
    # 結果をcsvファイルで出力
    result_name = 'result_' + date_now.strftime('%Y-%m-%d--%H-%M-%S') + '.csv'
    path_result = ab_path_result / result_name
    with open(path_result, 'x', newline="") as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerows(result)
    
    # 結果をターミナルにも表示
    for s in result:
        print(s[0])
    
    return None


if __name__ == '__main__':
    func_example_1(threshold = 40,)