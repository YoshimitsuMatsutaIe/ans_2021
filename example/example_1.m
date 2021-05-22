function example_1()
% 例題1：ファイル操作
%   example_1_dataフォルダにあるcsvファイルの中で，変数xにある数値以上の値が...
%   含まれる物をリストアップするプログラムを作成してください．
%   結果はcsvファイルでexample_1_rezultフォルダに保存されるようにしてください．
%   ただし結果のファイル名には実行時刻が入るようにしてください．

%% 日時取得
dt = datetime('now');
DateString = datestr(dt,'yyyy-MM-dd-HH-mm-ss-FFF');

%% ファイル，フォルダのパス取得
currentFolder = pwd;  % 現在のフォルダの絶対パス
dataFolder = append(currentFolder, '/example_1_data');  % csvの保存先フォルダ
outputFolder = append(currentFolder, '/example_1_result');  % 結果の保存先フォルダ
cd example_1_data  % フォルダへ移動
csv_list = dir(append(dataFolder, '/*.csv'));  % ワイルドカードで~.csvの名前取得
cd ..  % 元のフォルダに戻る

%% 処理
threshold = 40;  % 閾値
output = [];  % 条件を満たすcavファイル名を放り込む配列．

for i = 1:1:length(csv_list)  % ファイルの数だけループ
    temp_table = readtable([csv_list(i).folder '/' csv_list(i).name]);  % csvをtableとして読み込み
    temp_table = rmmissing(temp_table);  % 欠損値削除
    
    if any(temp_table.x > threshold)  % 閾値判定
        output = [output; string(csv_list(i).name)];  % 結果の配列に入れる
    else
        % 何もしない
    end    
end

%% 結果をcsvファイル出力
output_name = append(DateString, '_output.csv');
output_path = append(outputFolder, '/', output_name);
writematrix(output, output_path)

end