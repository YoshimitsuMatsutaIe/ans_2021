using CPUTime
using DifferentialEquations
using Plots


# K = 10.0
# dx(x, t) = 1.0 * (1 - x[1]^2) * x[2] - x[1]  # 微分方程式を定義

# x0 = [1.0, 1.0]  # 初期値を設定
# tspan = (0.0, 1.0)  # 解く範囲を設定

# prob = ODEProblem(dx , x0 , tspan)
# sol = solve(prob)  # ソルバで解く

# println("解けた!")

# # plot
# plot(
#     sol,
#     linewidth=5,
#     xaxis="t",
#     yaxis="x",
#     label="solution"
# )

# # savefig("solution_of_example_2.png")  #  保存



# オイラー法で解く（自作）

K = 2.0
Δt = 1
time_span = 10.0
x0 = [1.0, 1.0]

dx(x, K) = K * (1 - x[1]^2) * x[2] - x[1]


sol = [x0]
global  x
x = x0
println(x)
for i in 0:Δt:time_span
    x += dx(x, K) * Δt
    println(x)
    push!(sol, x)
end
