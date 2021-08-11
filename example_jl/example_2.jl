using DifferentialEquations
using Plots
using CPUTime

function hoge()
     a = 3.0  # パラメータ

     dy(y, t, a) = a * y  #微分方程式を定義
     y0 = 1.0  # 初期値を設定
     tspan = (0.0, 1.0)  # 解く範囲を設定

     prob = ODEProblem(dy , y0 , tspan)
     sol = solve(prob)  # ソルバで解く

     # plot
     plot(
          sol,
          linewidth=5,
          xaxis="x",
          yaxis="y(x)",
          label="solution"
     )

     # savefig("solution_of_example_2.png")  #  保存
end

@time hoge()