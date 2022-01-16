using DifferentialEquations
using Plots

function main()
     a = 3.0  # パラメータ

     """
     微分方程式
     """
     dy(y, t, a) = a * y
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


"""
オイラー法

・参考にしました -> [url](https://twitter.com/genkuroki/status/1301832571131633665/photo/1)  
"""
function jisaku_solve_euler(dx, x₀, t_span, Δt)
     t = range(t_span..., step = Δt)  # 時間軸
     x = Vector{typeof(x₀)}(undef, length(t))  # 解を格納する1次元配列

     x[1] = x₀  # 初期値
     for i in 1:length(x)-1
          x[i+1] = x[i] + dx(t[i], x[i])Δt
     end

     t, x
end

"""ルンゲクッタ法（4次）"""
function jisaku_solve_RungeKutta(dx, x₀, t_span, Δt)
     t = range(t_span..., step = Δt)  # 時間軸
     x = Vector{typeof(x₀)}(undef, length(t))  # 解を格納する1次元配列

     x[1] = x₀  # 初期値
     for i in 1:length(x)-1
          k₁ = dx(t[i], x[i])
          k₂ = dx(t[i]+Δt/2, x[i]+k₁*Δt/2)
          k₃ = dx(t[i]+Δt/2, x[i]+k₂*Δt/2)
          k₄ = dx(t[i]+Δt, x[i]+k₃*Δt)
          x[i+1] = x[i] + (k₁ + 2k₂ + 2k₃ +k₄)Δt/6
     end

     t, x
end


function main()
     a = 3.0  # パラメータ
     dx(t, x) = a * x  # 微分方程式

     x₀ = 1.0
     t_span = (0.0, 5.0)
     Δt = 0.01
     t, x = jisaku_solve_RungeKutta(dx, x₀, t_span, Δt)
     plot(t, x, label="RungeKutta")

     t, x = jisaku_solve_euler(dx, x₀, t_span, Δt)
     plot!(t, x, label="euler")

     x = exp.(a*t)
     plot!(t, x, label="exact sol exp(at)")
end


@time main()