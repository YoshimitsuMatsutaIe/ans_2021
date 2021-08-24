using CPUTime
using Plots
using DifferentialEquations
using LinearAlgebra
using MatrixEquations

# function ARE(A, B, Q, R)
#     """有本ポッターで解く（未完成）"""
#     ℋ = [
#         A' -B*inv(R)*B'
#         -Q -A
#     ]  #  ハミルトン行列
#     println("ℋ = ", ℋ)

#     λ_ = eigvals(ℋ)  # ハミルトン行列の固有値
#     ω_ = eigvecs(ℋ)  # ハミルトン行列の固有ベクトル
#     println("ℋ の固有値 = ", λ_)
#     println("ℋ の固有ベクトル", ω_)

#     # ハミルトン行列の固有値が負のものを探す
#     index = []
#     for i in 1:size(λ_)[1]
#         print(i)
#         if real(λ_[i]) < 0
#             println("実部が負")
#             push!(index, i)
#         else
#             println("実部が正")
#         end
#     end
#     println("ℋ の固有値の実部が負なのは, ", index)
#     n = size(index)[1]

#     # Y,Zを計算
#     ω = Matrix{ComplexF64}(undef, size(ω_)[1], n)  # 未初期化のMatrix

#     for i in 1:n
#         global ω[:, i] .= ω_[:, index[i]]
#     end

#     Y = ω[1:n, :]
#     Z = ω[n+1:end, :]

#     P = Z * inv(Y)
#     println("P = ", P)
#     return P
# end

function AREbyMatrixEquations(A, B, Q, R)
    """ライブラリ使用
    https://juliapackages.com/p/matrixequations
    """
    S = zero(B)
    P, E, F = arec(A, B, R, Q, S)
    println("P = ", P)
    return P
end


function simu(A, B, C, D, Q, R, x0)
    """解を描写"""
    #P = ARE(A, B, Q, R)  # リカッチ方程式を解く
    P = AREbyMatrixEquations(A, B, Q, R)
    K = inv(R) * B' * P  # 最適フィードバックゲイン
    Ā = A - B * K
    println("Ā = ", Ā)
    println(eigvals(Ā))
    
    # 解
    x(t) = exp(Ā .* t) * x0  # 状態方程式の解
    u(t) = -K * x(t)  # 最適入力
    y(t) = C * x(t) + D * u(t)  # 出力方程式の解


    # Plot．システムの次元に合わせてplotの数を変更．
    plot(xlims=(0, 10), xaxis="time", )
    plot!(t -> x.(t)[1], label="x1")
    plot!(t -> x.(t)[2], label="x2")
    plot!(t -> x.(t)[3], label="x3")
end




### システムの例1 ###
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
D = [1 1 1]
# 重み行列
Q = diagm(0 => [1,1,1]) .* 1000
R = ones((1,1))
x0 = [1, 2, 3]


# ### システムの例2 ###
# A = [0.0 1.0; -10.0 -1.0]
# B = [0.0, 1.0]
# Q = [300 0; 0 60]
# R = ones((1, 1))
# C = ones((2, 2))
# D = B
# x0 = [1, 2]

@time simu(A, B, C, D, Q, R, x0)  # 実行