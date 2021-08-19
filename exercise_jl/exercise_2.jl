using CPUTime
using Plots
using StaticArrays
using ArraysOfArrays


function split_vec_of_arrays(u)
    vec.(u) |>
    x -> VectorOfSimilarVectors(x).data |>
    transpose |>
    VectorOfSimilarVectors
end


function jisaku_solve_euler(dx, x₀, t_span, Δt)
    """オイラー法
    ・参考にしました: https://twitter.com/genkuroki/status/1301832571131633665/photo/1
    """

    t = range(t_span..., step = Δt)  # 時間軸
    x = Vector{typeof(x₀)}(undef, length(t))  # 解を格納する1次元配列

    x[1] = x₀  # 初期値
    for i in 1:length(x)-1
        x[i+1] = x[i] + dx(t[i], x[i])*Δt
    end

    t, x
end


function jisaku_solve_RungeKutta(dx, x₀, t_span, Δt)
    """ルンゲクッタ法（4次）"""

    t = range(t_span..., step = Δt)  # 時間軸
    x = Vector{typeof(x₀)}(undef, length(t))  # 解を格納する1次元配列

    x[1] = x₀  # 初期値
    for i in 1:length(x)-1
        k₁ = dx(t[i], x[i])
        k₂ = dx(t[i]+Δt/2, x[i]+k₁/2)
        k₃ = dx(t[i]+Δt/2, x[i]+k₂/2)
        k₄ = dx(t[i]+Δt, x[i]+k₃)
        x[i+1] = x[i] + 1/6*(k₁ + 2k₂ + 2k₃ +k₄)*Δt
    end

    t, x
end


function VanDerPol(t, x)
    K = 1.0
    [
        x[2]
        K * (1 - x[1]^2) * x[2] - x[1]
    ]
end


# 数値シミュレーション実行
x₀ = [0.1, 0.1]  # 初期値
t_span = (0.0, 50.0)  # 時間幅
Δt = 0.01  # 刻み時間
#t, x = jisaku_solve_euler(VanDerPol, x₀, t_span, Δt)  # 解く
@time t, x = jisaku_solve_RungeKutta(VanDerPol, x₀, t_span, Δt)  # 解く

# plot
x, v = split_vec_of_arrays(x)  # 解をplotしやすいように分割
fig1 = plot(
    t, x,
    #linewidth=3,
    xaxis="time",
    label="x",
)
plot!(fig1, t, v, label = "ẋ")

fig2 = plot(
    x, v,
    #linewidth=3,
    xaxis="x",
    yaxis="dx",
    label="trajectory",
)

fig3 = plot(
    t, x, v,
    xaxis="time", yaxis="x", zaxis="v",
    label="trajectory"
)

plot(
    fig1, fig2, fig3, layout=(3,1),
    size=(600,900)
)