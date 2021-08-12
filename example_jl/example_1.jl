using CPUTime
using CSV
using DataFrames

path = "./example_jl/example_1_data"  # csvフォルダのパス


cd(path)  # pathに移動
file_name_all = readdir()  # カレントディレクトリにある全ファイル名取得
csv_name = file_name_all[
    map(x->occursin(r"\.csv$",x),file_name_all)
]  # csvの名前だけ取得
println(csv_name)

for name in csv_name
    println(name)
    df =CSV.read(name, DataFrame)
    println(df[df.x .> 40, :])
    # if size(df[df.x .> 40, :]):> 0
    #     println("yes")
    # else
    #     println("no")
    # end
end

