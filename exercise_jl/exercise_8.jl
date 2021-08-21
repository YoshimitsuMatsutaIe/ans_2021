using Plots
using LinearAlgebra
using MatrixEquations
using CPUTime
using ArraysOfArrays


function split_vec_of_arrays(u)
    vec.(u) |>
    x -> VectorOfSimilarVectors(x).data |>
    transpose |>
    VectorOfSimilarVectors
end


function jisaku_solve_RungeKutta(dx, x₀, t_span, Δt, args)
    """ルンゲクッタ法（4次）"""

    t = range(t_span..., step = Δt)  # 時間軸
    x = Vector{typeof(x₀)}(undef, length(t))  # 解を格納する1次元配列

    x[1] = x₀  # 初期値
    for i in 1:length(x)-1
        k₁ = dx(t[i], x[i], args)
        k₂ = dx(t[i]+Δt/2, x[i]+k₁*Δt/2, args)
        k₃ = dx(t[i]+Δt/2, x[i]+k₂*Δt/2, args)
        k₄ = dx(t[i]+Δt, x[i]+k₃*Δt, args)
        x[i+1] = x[i] + (k₁ + 2k₂ + 2k₃ +k₄)Δt/6
    end

    t, x
end


function dx(t, x, param)
    """微分方程式"""

    M, m, L, l, D, d, g, K = param

    multi = [
        1 0 0 0
        0 0 1 0
        0 M+m 0 m*l*cos(x[3])
        0 m*l*cos(x[3]) 0 4/3*m*l^2
    ]
    offset_x = [
        x[2]
        x[4]
        m*l*x[4]^2*sin(x[3])-D*x[2]
        m*g*l*sin(x[3])-d*x[4]
    ]
    offset_u = [
        0
        0
        1
        0
    ]

    Fa = inv(multi) * offset_x
    Fb = inv(multi) * offset_u

    u = -K*x
    
    dx = Fa + Fb.*u
end


function ByLQR(M, m, L, l, D, d, g,)
    """LQRで制御"""

    # 原点近傍での線形化システム
    multi = [
        1 0 0 0
        0 0 1 0
        0 M + m 0 m * L
        0 m * L 0 4/3 * m * l^2
    ]
    offset_x = [
        0 1 0 0
        0 0 0 1
        0 -D 0 0
        0 0 m * g * l -d
    ]
    offset_u = [
        0
        0
        1
        0
    ]

    A = inv(multi) * offset_x
    B = inv(multi) * offset_u
    
    # リカッチ方程式を解く
    Q = diagm(0 => [10,10,100,100])  # 重み行列
    R = ones((1,1)) .* 0.01  # 重み行列

    S = zero(B)
    P, _, _ = arec(A, B, R, Q, S)

    K = inv(R) * B' * P  # 最適フィードバックゲイン
    #println(K)

    ### 数値シミュレーション ###
    param = M, m, L, l, D, d, g, K
    x0 = [0, 0, pi/6, 0]
    tspan = (0.0, 5.0)  # 範囲を設定
    Δt = 0.01
    t, x = jisaku_solve_RungeKutta(dx, x0, tspan, Δt, param)
    x, v, θ, ω = split_vec_of_arrays(x)

    # plot
    #plot([x, v, θ, ω], label=["x" "v" "θ" "ω"])

    anim = @animate for i in 1:2:length(t)
        now = round(i * Δt, digits=1)
        s = "t = " * string(now) * " [s]"
        plot(
            [x[i], x[i]+L*cos(-θ[i]+π/2)],
            [0, L*sin(-θ[i]+π/2)],
            #size=(400,400),
            linewidth=5,
            xlims = (-3L, 3L),
            ylims = (-1.1L, 1.1L),
            aspect_ratio = 1,
            legend=false,
            annotate=(0, -1, s),
        )
    end
    gif(anim, "example_8_jl.gif", fps = 30)
    #mp4(anim, "example_8_jl.mp4", fps = 30)

end


# パラメータ
M = 5.0  # 車両質量[kg]
m = 1.0  # 振子質量[kg]
L = 1.5  # 振子長さ[m]
l = L/2
D = 0.01  # 車両と地面の動摩擦係数
d = 0.01  # 振子と車両の動摩擦係数
g = 9.80665  # 重力加速度[m/s²]


@time ByLQR(M, m, L, l, D, d, g)


