using CPUTime
using CSV
using DataFrames
using Dates


function do_example_1(path, threshold)

    now = Dates.now(Dates.UTC)

    cd(path)  # pathに移動
    file_name_all = readdir()  # カレントディレクトリにある全ファイル名取得
    csv_name = file_name_all[
        map(x->occursin(r"\.csv$",x),file_name_all)
    ]  # csvの名前だけ取得

    result = []

    # 条件を満たすcsvファイルを探す
    for name in csv_name
        df = CSV.read(name, DataFrame)
        if size(df[df.x .> threshold, :], 1) > 0
            push!(result, name)
        end
    end

    cd("..")
    if isdir("./example_rsult")
        ##
    else
        mkdir("./example_rsult")
    end
    cd("./example_rsult")

    # 結果をtxtで出力
    dt = Dates.format(now, "yyyy-mm-dd--HH-MM-SS")
    result_name = string(dt) * "_result.txt"

    open(result_name, "w+") do f
        print(f, "result is")
        for name in result
            println(f, name)
        end
    end

    cd("..")
    cd("..")
end



path = "./exeicise_jl/exercise_1_data"  # csvフォルダのパス
threshold = 40  # xの閾値

@time do_example_1(path, threshold)