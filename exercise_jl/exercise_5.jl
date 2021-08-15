using CPUTime
using Plots
using DifferentialEquations
using LinearAlgebra


function LQR(A, B, Q, R)
    """有本ポッターで解く"""
    ℋ = [
        A' -B*inv(R)*B'
        -Q -A
    ]  #  ハミルトン行列

    λ_ = eigvals(ℋ)  # ハミルトン行列の固有値
    ω_ = eigvecs(ℋ)  # ハミルトン行列の固有ベクトル

    index = findall(λ_.<0)  # ハミルトン行列の固有値が正のものを探す
    n = size(index)[1]

    # Y,Zを計算
    ω = Matrix{Float64}(undef, size(ω_)[1], n)  # 未初期化のMatrix

    for i in 1:n
        global ω[:, i] .= ω_[:, index[i]]
    end

    Y = ω[1:n, :]
    Z = ω[n+1:end, :]

    P = Z * inv(Y)
    println("P = ", P)
    return P
end


function simu(A, B, C, D, Q, R, x0)
    """解を描写"""
    P = LQR(A, B, Q, R)  # リカッチ方程式を解く
    K = inv(R) * B' * P  # 最適フィードバックゲイン
    Ā = A - B * K
    println("Ā = ", Ā)
    println(eigvals(Ā))
    println("OK")
    # 解
    x(t) = exp(Ā .* t) * x0
    # u(t) = -K * x(t)
    # y(t) = C * x(t) + D * u(t)
    #println(x(10))
    # Plot
    x1(t) = x(t)[1]
    plot(x1, 0, 10)
end





A = [
    1.1 2.0 3.0
    0 0.95 1.20
    1.2 0.01 10.5
]
B = [
    1.0
    0.0
    0.847
]
C = Matrix(I, 2, 2)

# 重み行列
Q = diagm(0 => [1,1,1]) .* 1000
R = ones((1,1))

D = [1 1 1]

x0 = [1, 2, 3]

@time simu(A, B, C, D, Q, R, x0)