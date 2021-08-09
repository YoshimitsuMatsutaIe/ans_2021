using DifferentialEquations
using Plots


f(y,t) = 3.0*y  #微分方程式を定義
y0 = 1.0  # 初期値を設定
tspan = (0.0 , 1.0)  # 解く範囲を設定

prob = ODEProblem(f , y0 , tspan)
sol = solve(prob)  # ソルバで解く

# plot
plot(
     sol,
     linewidth=5,
     title="Solution",
     xaxis="x",
     yaxis="y(x)",
     label="solution"
)
