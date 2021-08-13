using CPUTime
using Plots


function jisaku_euler()
    """オイラー法で解く（自作）"""
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

    fig1 = plot(
        sol_x,
        linewidth=3,
        xaxis="time",
        label="x",
    )

    plot!(fig1, sol_dx, label = "dx")

    fig2 = plot(
        sol_x,
        sol_dx,
        linewidth=3,
        xaxis="x",
        yaxis="dx",
        label="trajectory",
    )

    plot(fig1, fig2, layout=(2,1),size=(600,900))

end






@time jisaku_euler()