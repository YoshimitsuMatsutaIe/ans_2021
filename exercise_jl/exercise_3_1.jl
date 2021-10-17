using CPUTime
using Plots
using DifferentialEquations

f = @ode_def begin
    dx = v
    dv = -4(-D*v + ω^2*l*m*sin(θ))/(-4M + 3m*cos(θ)^2 - 4m) +
    3cos(θ)*(-d + ω + g*l*m*sin(θ))/(l*(-4M + 3m*cos(θ)^2 -4m)) +
    -4/(-4M + 3m*cos(θ)^2 -4m)
    dθ = ω
    dω = 3cos(θ)*(D*v + ω^2*l*m*sin(θ))/(l*(-4M + 3m*cos(θ)^2 -4*m)) +
    -3(M + m)*(-d + ω + g*l*m*sin(θ))/(l^2*(-4M*m + 3m^2*cos(θ)^2 - 4m^2)) +
    3cos(θ)/(l*(-4M + 3m*cos(θ)^2 - 4m))
end M m L l D d g


function do_3_1()
    # パラメータ
    M = 5.0  # 車両質量[kg]
    m = 1.0  # 振子質量[kg]
    L = 1.5  # 振子長さ[m]
    l = L/2
    D = 0.01  # 車両と地面の動摩擦係数
    d = 0.01  # 振子と車両の動摩擦係数
    g = 9.80665  # 重力加速度[m/s²]
    p = [M, m, L, l, D, d, g]

    x0 = [0.0, 0.0, π/6, 0.0]  # 初期値を設定
    tspan = (0.0, 3.0)  # 解く範囲を設定

    prob = ODEProblem(f ,x0 ,tspan, p)
    sol = solve(prob)  # ソルバで解く

    anim = @animate for i in 1:length(sol.t)
        now = round(sol.t[i], digits=1)
        s = "t = " * string(now) * " [s]"
        x, v, θ, ω = sol.u[i]
        plot(
            [x, x+L*cos(-θ+π/2)],
            [0, L*sin(-θ+π/2)],
            #size=(400,400),
            linewidth=5,
            xlims = (-3L, 3L),
            ylims = (-1.1L, 1.1L),
            aspect_ratio = 1,
            legend=false,
            annotate=(0, -1, s),
        )
    end
    gif(anim, "example_3_1_jl.gif", fps = 30)


    return sol, anim

end


@time sol, anim = do_3_1()