using CPUTime
using DifferentialEquations
using Plots

f = @ode_def begin
    dx = v
    dv = 1/m*(-c*v - k*x + u)
    du = -Kp*v + Ki*(xg-x) - Kd/m*(-c*v - k*x + u)
end m c k xg Kp Ki Kd



function do_1()
    """1つめのやつ"""
    m = 1.0
    c = 1.0
    k = 1.0
    xg = 1.0
    Kp = 3
    Ki = 1
    Kd = 0.05

    p = [m, c, k, xg, Kp, Ki, Kd]

    x0 = [0.0, 0.0, Kp*(xg - 1.0)]  # 初期値を設定
    tspan = (0.0, 20.0)  # 解く範囲を設定

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


    n = 1000  # アニメの枚数
    Kp_min, Kp_max = 9.99, 10.0
    Ki_min, Ki_max = 4.99, 5.0
    Kd_min, Kd_max = 0.0, 5.0
    Kp = [i for i in Kp_min:(Kp_max-Kp_min)/n:Kp_max]
    Ki = [i for i in Ki_min:(Ki_max-Ki_min)/n:Ki_max]
    Kd = [i for i in Kd_min:(Kd_max-Kd_min)/n:Kd_max]

    sols = []
    tspan = (0.0, 5.0)  # 解く範囲を設定

    for j in 1:n
        x0 = [0.0, 0.0, Kp[j]*(xg - 1.0)]  # 初期値を設定
        p = [m, c, k, xg, Kp[j], Ki[j], Kd[j]]
        prob = ODEProblem(f ,x0 ,tspan, p)
        sol = solve(prob)  # ソルバで解く
        push!(sols, sol)
    end

    ano_xposi = (tspan[1] + tspan[2])/2
    anim = @animate for i in 1:1:n #length(sols)
        Kp_now = round(Kp[i], digits=1)
        Ki_now = round(Ki[i], digits=1)
        Kd_now = round(Kd[i], digits=1)
        s = "Kp = " * string(Kp_now) * ", Ki = " * string(Ki_now) * ", Kd = " * string(Kd_now)
        plot(
            sols[i],
            ylims = (-1, xg*1.5), annotate = (ano_xposi, -1, s),
        )
    end
    mp4(anim, "example_4_jl.mp4", fps = 30)



end


#@time do_1()

@time do_2()