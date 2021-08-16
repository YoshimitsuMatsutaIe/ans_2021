using Plots
using DifferentialEquations
using MatrixEquations
using CPUTime

struct InvertedPendulumParameter
    M::Float64
    m::Float64
    L::Float64
    l::Float64
    D::Float64
    d::Float64
    g::Float64
end


function dx(t, x, input, param)
    """微分方程式"""

    # M = 1.0
    # m = 1.0
    # L = 1.0
    # l = L/2
    # D = 1.0
    # d = 1.0
    # g = 9.8


    multi = [
        1 0 0 0
        0 0 1 0
        0 param.M+param.m 0 param.m*param.l*cos(x[3])
        0 param.m*param.l*cos(x[3]) 0 4/3*param.m*param.l^2
    ]
    offset_x = [
        x[2]
        x[4]
        param.m*param.l*x[4]^2*sin(x[3])-param.D*x[2]
        param.m*param.g*param.l*sin(x[3])-param.d*x[4]
    ]
    offset_u = [
        0
        0
        1
        0
    ]

    u = input(x)

    Fa = inv(multi) * offset_x
    Fb = inv(multi) * offset_u

    dx = Fa + Fb*u
end


function ByLQR(x0=[0 0 π/6 0], param)
    """LQRで制御"""


    Δt = 0.01
    time_span = 10

    # 原点近傍での線形化システム
    multi = [
        1 0 0 0
        0 0 1 0
        0 param.M + param.m 0 param.m * param.L
        0 param.m * param.L 0 4/3 * param.m * param.l^2
    ]
    offset_x = [
        0 1 0 0
        0 0 0 1
        0 -param.D 0 0
        0 0 param.m * param.g * param.l -param.d
    ]
    offset_u = [
        0 0 1 0
    ]

    A = inv(multi) * offset_x
    B = inv(multi) * offset_u


    # リカッチ方程式を解く
    Q = diagm(0 => [1,1,1,1]) .* 100  # 重み行列
    R = ones((1,1))  # 重み行列

    S = zero(B)
    P, _, _ = arec(A, B, R, Q, S)

    function input(x)
        



