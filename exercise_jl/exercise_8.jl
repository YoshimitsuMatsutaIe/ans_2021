using Plots
using LinearAlgebra
using MatrixEquations
using CPUTime


function split_vec_of_arrays(u)
    vec.(u) |>
    x -> VectorOfSimilarVectors(x).data |>
    transpose |>
    VectorOfSimilarVectors
end


function jisaku_solve_euler(dx, x₀, t_span, Δt, args)
    """オイラー法
    ・参考にしました: https://twitter.com/genkuroki/status/1301832571131633665/photo/1
    """

    t = range(t_span..., step = Δt)  # 時間軸
    x = Vector{typeof(x₀)}(undef, length(t))  # 解を格納する1次元配列

    x[1] = x₀  # 初期値
    for i in 1:length(x)-1
        x[i+1] = x[i] + dx(t[i], x[i], args)*Δt
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
    println(K)

    ### 数値シミュレーション ###
    param = M, m, L, l, D, d, g, K
    x0 = [0, 0, pi/6, 0]
    tspan = (0.0, 5.0)  # 範囲を設定
    t, x = jisaku_solve_euler(dx, x0, tspan, 0.01, param)
    x, v, θ, ω = split_vec_of_arrays(x)

    # plot
    #plot([x, v, θ, ω], label=["x" "v" "θ" "ω"])
    anim = @animate for i in 1:length(t)
        plot(
            [x[i], x[i]+L*cos(-θ[i]+π/2)],
            [0, L*sin(-θ[i]+π/2)],
            #size=(400,400),
            linewidth=5,
            xlims = (-3L, 3L),
            ylims = (-1.1L, 1.1L),
            aspect_ratio = 1,
            legend=false,
        )
    end
    gif(anim, "ex.gif", fps = 30)

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


