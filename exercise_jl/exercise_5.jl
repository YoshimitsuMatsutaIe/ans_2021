using CPUTime
using Plots
using DifferentialEquations
using LinearAlgebra


# 有本ポッターで解く

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


ℋ = [
    A' -B*inv(R)*B'
    -Q -A
]  #  ハミルトン行列


λ = eigvals(ℋ)  # ハミルトン行列の固有値
ω = eigvecs(ℋ)  # ハミルトン行列の固有ベクトル



print(λ)