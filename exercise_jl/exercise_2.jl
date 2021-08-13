using CPUTime
#using DifferentialEquations
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
function jisaku_euler()
    K = 1.0
    Δt = 0.001
    time_span = 50.0
    x0 = [0.1, 0.1]

    dx(x, K) = [x[2], K * (1 - x[1]^2) * x[2] - x[1]]


    sol_x = []
    sol_dx = []

    for i in 0:Δt:time_span
        global x
        if i ==0
            x = x0
        end
        x = x + dx(x, K) * Δt
        push!(sol_x, x[1])
        push!(sol_dx, x[2])
    end

    plot(
        sol_x,
        sol_dx,
        linewidth=3,
        xaxis="x",
        yaxis="dx",
        label="trajectory",
    )
end

@time jisaku_euler()