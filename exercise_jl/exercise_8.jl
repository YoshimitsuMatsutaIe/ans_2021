using Plots
using DifferentialEquations
using LinearAlgebra
using MatrixEquations
using CPUTime

mutable struct InvertedPendulumParameter
    M::Float64
    m::Float64
    L::Float64
    l::Float64
    D::Float64
    d::Float64
    g::Float64
end


function dx(x, p, t)
#function dx(x, t, M, m, L, l, D, d, g)
    """微分方程式"""
    M = 5.0
    m = 1.0
    L = 1.5
    l = L/2
    D = 0.01
    d = 0.01
    g = 9.80665

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

    #u = input(x, input_param)
    u = 0.0

    Fa = inv(multi) * offset_x
    Fb = inv(multi) * offset_u

    dx = Fa + Fb*u
end


function ByLQR(M, m, L, l, D, d, g, x0=[0 0 2 0], )

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
    Q = diagm(0 => [1,1,1,1]) .* 100  # 重み行列
    R = ones((1,1))  # 重み行列

    S = zero(B)
    P, _, _ = arec(A, B, R, Q, S)

    K = inv(R) * B' * P  # 最適フィードバックゲイン

    function input(x, K)
        return u = -K * x
    end

    ### 数値シミュレーション ###
    param = M, m, L, l, D, d, g
    println(param)
    tspan = (0.0, 1.0)  # 範囲を設定
    prob = ODEProblem(dx, x0, tspan)  # 問題を設定
    sol = solve(prob)  # ソルバで解く

    # plot
    plot(sol)
end


M = 1.0
m = 1.0
L = 1.0
l = L/2
D = 1.0
d = 1.0
g = 9.8


dx0 = dx([0 0 2 0], 0, 0)



ByLQR(M, m, L, l, D, d, g)







