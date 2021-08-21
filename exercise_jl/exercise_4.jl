using CPUTime
using DifferentialEquations
using Plots

f = @ode_def begin
    dv = 1/m*(-c*v - k*x + u)
    dx = v
    du = -Kp*u + Ki*(xg-x) - Kd/m*(-c*v - k*x + u)
end m c k xg Kp Ki Kd



function do_1()
    """1つめのやつ"""
    m = 1.0
    c = 1.0
    k = 1.0
    xg = 0.0
    Kp = 3
    Ki = 1
    Kd = 0.05

    p = [m, c, k, xg, Kp, Ki, Kd]

    x0 = [1.0, 0.0, Kp*(xg - 1.0)]  # 初期値を設定
    tspan = (0.0, 10.0)  # 解く範囲を設定

    prob = ODEProblem(f ,x0 ,tspan, p)
    sol = solve(prob)  # ソルバで解く

    # plot
    plot(
        sol,
        linewidth=2,
        xaxis="time [sec]",
        yaxis="",
        label=["position" "velocity" "input"]
    )

end


function do_2()
    """2つめのやつ"""
    m = 1.0
    c = 1.0
    k = 1.0
    xg = 1.0


    n = 50  # アニメの枚数
    K_min, K_max = 0.0, 3.0
    Kp = [i for i in K_min:(K_max-K_min)/n:K_max]
    Ki = 5.0
    Kd = 0.05

    sols = []
    tspan = (0.0, 10.0)  # 解く範囲を設定

    for j in 1:n
        x0 = [0.0, 0.0, Kp[j]*(xg - 1.0)]  # 初期値を設定
        p = [m, c, k, xg, Kp[j], Ki, Kd]
        prob = ODEProblem(f ,x0 ,tspan, p)
        sol = solve(prob)  # ソルバで解く
        push!(sols, sol)
    end

    anim = @animate for i in 1:1:length(sols)
        now = round(Kp[i], digits=1)
        s = "Kp = " * string(now) * ", Ki = " * string(Ki) * ", Kd = " * string(Kd)
        plot(
            sols[i],
            ylims = (-5, 5), annotate = (5, -1, s),
        )
    end
    mp4(anim, "example_4_jl.mp4", fps = 60)



end


#@time do_1()

@time do_2()