using CPUTime
using Plots
using DifferentialEquations
using LinearAlgebra
using Random

function origins(q)
    """世界座標系から見た局所座標系の原点座標をまとめて計算"""

    L = 278e-3
    h = 64e-3
    H = 1104e-3
    L0 = 270.35e-3
    L1 = 69e-3
    L2 = 364.35e-3
    L3 = 69e-3
    L4 = 374.29e-3
    L5 = 10e-3
    L6 = 368.3e-3

    o_Wo = [
        0
        0
        0
    ]

    o_BL = [
        L
        -h
        H
    ]

    o_0 = [
        L
        -h
        H + L0
    ]

    o_1 = [
        L
        -h
        H + L0
    ]

    o_2 = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1])
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) - h
        H + L0
    ]

    o_3 = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2])
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) - 0.707106781186548*L2*cos(q[1])*cos(q[2]) - h
        H + L0 - L2*sin(q[2])
    ]

    o_4 = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) - 0.707106781186548*L2*cos(q[1])*cos(q[2]) - 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) - h
        H + L0 - L2*sin(q[2]) - L3*cos(q[2])*cos(q[3])
    ]

    o_5 = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4]))
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) - 0.707106781186548*L2*cos(q[1])*cos(q[2]) - 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) + 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) - h
        H + L0 - L2*sin(q[2]) - L3*cos(q[2])*cos(q[3]) - L4*(sin(q[2])*cos(q[4]) + sin(q[4])*cos(q[2])*cos(q[3]))
    ]

    o_6 = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5]))
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) - 0.707106781186548*L2*cos(q[1])*cos(q[2]) - 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) + 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5])) - h
        H + L0 - L2*sin(q[2]) - L3*cos(q[2])*cos(q[3]) - L4*(sin(q[2])*cos(q[4]) + sin(q[4])*cos(q[2])*cos(q[3])) + L5*((sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[2]))
    ]

    o_7 = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5]))
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) - 0.707106781186548*L2*cos(q[1])*cos(q[2]) - 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) + 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5])) - h
        H + L0 - L2*sin(q[2]) - L3*cos(q[2])*cos(q[3]) - L4*(sin(q[2])*cos(q[4]) + sin(q[4])*cos(q[2])*cos(q[3])) + L5*((sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[2]))
    ]

    o_GL = [
        L + 0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5])) + L6*(0.707106781186548*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5]))*sin(q[6]) + 0.707106781186548*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + 0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) + cos(q[1])*cos(q[2])*cos(q[4]))*cos(q[6]) + 0.707106781186548*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*cos(q[2])*cos(q[4]))*cos(q[6]))
        0.707106781186548*L1*sin(q[1]) - 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) - 0.707106781186548*L2*cos(q[1])*cos(q[2]) - 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1])) + 0.707106781186548*L4*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5])) + L6*(-0.707106781186548*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5]))*sin(q[6]) + 0.707106781186548*(((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) - 0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) + cos(q[1])*cos(q[2])*cos(q[4]))*cos(q[6]) + 0.707106781186548*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*cos(q[2])*cos(q[4]))*cos(q[6])) - h
        H + L0 - L2*sin(q[2]) - L3*cos(q[2])*cos(q[3]) - L4*(sin(q[2])*cos(q[4]) + sin(q[4])*cos(q[2])*cos(q[3])) + L5*((sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[2])) + L6*(((sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[2]))*sin(q[6]) + (-sin(q[2])*cos(q[4]) - sin(q[4])*cos(q[2])*cos(q[3]))*cos(q[6]))
    ]

    return [o_Wo o_BL o_0 o_1 o_2 o_3 o_4 o_5 o_6 o_7 o_GL]
end

# ヤコビ行列
function jacobi_GL(q)
    L = 278e-3
    h = 64e-3
    H = 1104e-3
    L0 = 270.35e-3
    L1 = 69e-3
    L2 = 364.35e-3
    L3 = 69e-3
    L4 = 374.29e-3
    L5 = 10e-3
    L6 = 368.3e-3
    z = [
        -0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) - 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) + 0.707106781186548*L3*(sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1])) - 0.707106781186548*L4*((sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) - 0.707106781186548*L4*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*cos(q[2])*cos(q[4])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) + 0.707106781186548*L5*(((sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*sin(q[2])*sin(q[3]) - cos(q[1])*cos(q[3]))*sin(q[5])) + L6*((0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5]))*sin(q[6]) + (0.707106781186548*((sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + 0.707106781186548*(-sin(q[1])*sin(q[2])*sin(q[3]) - cos(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (0.707106781186548*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) + 0.707106781186548*cos(q[1])*cos(q[2])*cos(q[4]))*cos(q[6]) + (0.707106781186548*(sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) - 0.707106781186548*sin(q[1])*cos(q[2])*cos(q[4]))*cos(q[6])) -0.707106781186548*L2*sin(q[1])*sin(q[2]) - 0.707106781186548*L2*sin(q[2])*cos(q[1]) - 0.707106781186548*L3*sin(q[1])*cos(q[2])*cos(q[3]) - 0.707106781186548*L3*cos(q[1])*cos(q[2])*cos(q[3]) - 0.707106781186548*L4*(sin(q[1])*sin(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3])) - 0.707106781186548*L4*(sin(q[2])*cos(q[1])*cos(q[4]) + sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])) + 0.707106781186548*L5*((sin(q[1])*sin(q[2])*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[1])*sin(q[3])*sin(q[5])*cos(q[2])) + 0.707106781186548*L5*((sin(q[2])*sin(q[4])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[1])*cos(q[2])) + L6*((0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + 0.707106781186548*sin(q[1])*sin(q[3])*sin(q[5])*cos(q[2]))*sin(q[6]) + (0.707106781186548*(sin(q[2])*sin(q[4])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + 0.707106781186548*sin(q[3])*sin(q[5])*cos(q[1])*cos(q[2]))*sin(q[6]) + (-0.707106781186548*sin(q[1])*sin(q[2])*cos(q[4]) - 0.707106781186548*sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]))*cos(q[6]) + (-0.707106781186548*sin(q[2])*cos(q[1])*cos(q[4]) - 0.707106781186548*sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]))*cos(q[6])) 0.707106781186548*L3*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])) + 0.707106781186548*L3*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3])) - 0.707106781186548*L4*(sin(q[1])*cos(q[3]) - sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[4]) - 0.707106781186548*L4*(-sin(q[1])*sin(q[2])*sin(q[3]) - cos(q[1])*cos(q[3]))*sin(q[4]) + 0.707106781186548*L5*((sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[4])*cos(q[5])) + 0.707106781186548*L5*((sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[4])*cos(q[5]) + (sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[5])) + L6*((0.707106781186548*(sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[5]) + 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[4])*cos(q[5]))*sin(q[6]) + 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[4])*cos(q[6]) + (0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[4])*cos(q[5]) + 0.707106781186548*(sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[5]))*sin(q[6]) + 0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[4])*cos(q[6])) -0.707106781186548*L4*((sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) + sin(q[4])*cos(q[1])*cos(q[2])) - 0.707106781186548*L4*((sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])) + 0.707106781186548*L5*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4]))*cos(q[5]) + 0.707106781186548*L5*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4]))*cos(q[5]) + L6*(0.707106781186548*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4]))*sin(q[6])*cos(q[5]) + (0.707106781186548*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - 0.707106781186548*sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[6]) + 0.707106781186548*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4]))*sin(q[6])*cos(q[5]) + (0.707106781186548*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - 0.707106781186548*sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[6])) 0.707106781186548*L5*(-((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*sin(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[5])) + 0.707106781186548*L5*(-((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*sin(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[5])) + L6*((-0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*sin(q[5]) + 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[5]))*sin(q[6]) + (-0.707106781186548*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*sin(q[5]) + 0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[5]))*sin(q[6])) L6*((0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5]))*cos(q[6]) + (0.707106781186548*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + 0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5]))*cos(q[6]) - (0.707106781186548*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) + 0.707106781186548*cos(q[1])*cos(q[2])*cos(q[4]))*sin(q[6]) - (0.707106781186548*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) + 0.707106781186548*sin(q[1])*cos(q[2])*cos(q[4]))*sin(q[6])) 0
        0.707106781186548*L1*sin(q[1]) + 0.707106781186548*L1*cos(q[1]) + 0.707106781186548*L2*sin(q[1])*cos(q[2]) + 0.707106781186548*L2*cos(q[1])*cos(q[2]) + 0.707106781186548*L3*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3])) - 0.707106781186548*L3*(sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1])) - 0.707106781186548*L4*((sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4])) + 0.707106781186548*L4*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*cos(q[2])*cos(q[4])) + 0.707106781186548*L5*(((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5])) - 0.707106781186548*L5*(((sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + (-sin(q[1])*sin(q[2])*sin(q[3]) - cos(q[1])*cos(q[3]))*sin(q[5])) + L6*((0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) + 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5]))*sin(q[6]) + (-0.707106781186548*((sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) - 0.707106781186548*(-sin(q[1])*sin(q[2])*sin(q[3]) - cos(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (0.707106781186548*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) + 0.707106781186548*cos(q[1])*cos(q[2])*cos(q[4]))*cos(q[6]) + (-0.707106781186548*(sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) + 0.707106781186548*sin(q[1])*cos(q[2])*cos(q[4]))*cos(q[6])) -0.707106781186548*L2*sin(q[1])*sin(q[2]) + 0.707106781186548*L2*sin(q[2])*cos(q[1]) - 0.707106781186548*L3*sin(q[1])*cos(q[2])*cos(q[3]) + 0.707106781186548*L3*cos(q[1])*cos(q[2])*cos(q[3]) - 0.707106781186548*L4*(sin(q[1])*sin(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3])) + 0.707106781186548*L4*(sin(q[2])*cos(q[1])*cos(q[4]) + sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])) + 0.707106781186548*L5*((sin(q[1])*sin(q[2])*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[1])*sin(q[3])*sin(q[5])*cos(q[2])) - 0.707106781186548*L5*((sin(q[2])*sin(q[4])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[1])*cos(q[2])) + L6*((0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + 0.707106781186548*sin(q[1])*sin(q[3])*sin(q[5])*cos(q[2]))*sin(q[6]) + (-0.707106781186548*(sin(q[2])*sin(q[4])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) - 0.707106781186548*sin(q[3])*sin(q[5])*cos(q[1])*cos(q[2]))*sin(q[6]) + (-0.707106781186548*sin(q[1])*sin(q[2])*cos(q[4]) - 0.707106781186548*sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]))*cos(q[6]) + (0.707106781186548*sin(q[2])*cos(q[1])*cos(q[4]) + 0.707106781186548*sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]))*cos(q[6])) -0.707106781186548*L3*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])) + 0.707106781186548*L3*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3])) + 0.707106781186548*L4*(sin(q[1])*cos(q[3]) - sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[4]) - 0.707106781186548*L4*(-sin(q[1])*sin(q[2])*sin(q[3]) - cos(q[1])*cos(q[3]))*sin(q[4]) - 0.707106781186548*L5*((sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[4])*cos(q[5])) + 0.707106781186548*L5*((sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[4])*cos(q[5]) + (sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[5])) + L6*((-0.707106781186548*(sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[5]) - 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[4])*cos(q[5]))*sin(q[6]) - 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[4])*cos(q[6]) + (0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[4])*cos(q[5]) + 0.707106781186548*(sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[5]))*sin(q[6]) + 0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[4])*cos(q[6])) 0.707106781186548*L4*((sin(q[1])*sin(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) + sin(q[4])*cos(q[1])*cos(q[2])) - 0.707106781186548*L4*((sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])) - 0.707106781186548*L5*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4]))*cos(q[5]) + 0.707106781186548*L5*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4]))*cos(q[5]) + L6*(-0.707106781186548*(-(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - cos(q[1])*cos(q[2])*cos(q[4]))*sin(q[6])*cos(q[5]) + (-0.707106781186548*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) + 0.707106781186548*sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[6]) + 0.707106781186548*(-(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*cos(q[2])*cos(q[4]))*sin(q[6])*cos(q[5]) + (0.707106781186548*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - 0.707106781186548*sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[6])) -0.707106781186548*L5*(-((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*sin(q[5]) + (-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[5])) + 0.707106781186548*L5*(-((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*sin(q[5]) + (sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[5])) + L6*((0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*sin(q[5]) - 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*cos(q[5]))*sin(q[6]) + (-0.707106781186548*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*sin(q[5]) + 0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*cos(q[5]))*sin(q[6])) L6*((-0.707106781186548*((-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2]))*cos(q[5]) - 0.707106781186548*(-sin(q[1])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1]))*sin(q[5]))*cos(q[6]) + (0.707106781186548*((-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2]))*cos(q[5]) + 0.707106781186548*(sin(q[1])*sin(q[2])*sin(q[3]) + cos(q[1])*cos(q[3]))*sin(q[5]))*cos(q[6]) - (-0.707106781186548*(-sin(q[1])*sin(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]))*sin(q[4]) - 0.707106781186548*cos(q[1])*cos(q[2])*cos(q[4]))*sin(q[6]) - (0.707106781186548*(-sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[3])*cos(q[1]))*sin(q[4]) + 0.707106781186548*sin(q[1])*cos(q[2])*cos(q[4]))*sin(q[6])) 0
        0 -L2*cos(q[2]) + L3*sin(q[2])*cos(q[3]) - L4*(-sin(q[2])*sin(q[4])*cos(q[3]) + cos(q[2])*cos(q[4])) + L5*((sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[4])*cos(q[2]))*cos(q[5]) - sin(q[2])*sin(q[3])*sin(q[5])) + L6*(((sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[4])*cos(q[2]))*cos(q[5]) - sin(q[2])*sin(q[3])*sin(q[5]))*sin(q[6]) + (sin(q[2])*sin(q[4])*cos(q[3]) - cos(q[2])*cos(q[4]))*cos(q[6])) L3*sin(q[3])*cos(q[2]) + L4*sin(q[3])*sin(q[4])*cos(q[2]) + L5*(sin(q[3])*cos(q[2])*cos(q[4])*cos(q[5]) + sin(q[5])*cos(q[2])*cos(q[3])) + L6*((sin(q[3])*cos(q[2])*cos(q[4])*cos(q[5]) + sin(q[5])*cos(q[2])*cos(q[3]))*sin(q[6]) + sin(q[3])*sin(q[4])*cos(q[2])*cos(q[6])) -L4*(-sin(q[2])*sin(q[4]) + cos(q[2])*cos(q[3])*cos(q[4])) + L5*(sin(q[2])*cos(q[4]) + sin(q[4])*cos(q[2])*cos(q[3]))*cos(q[5]) + L6*((sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[6]) + (sin(q[2])*cos(q[4]) + sin(q[4])*cos(q[2])*cos(q[3]))*sin(q[6])*cos(q[5])) L5*(-(sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*sin(q[5]) + sin(q[3])*cos(q[2])*cos(q[5])) + L6*(-(sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*sin(q[5]) + sin(q[3])*cos(q[2])*cos(q[5]))*sin(q[6]) L6*(((sin(q[2])*sin(q[4]) - cos(q[2])*cos(q[3])*cos(q[4]))*cos(q[5]) + sin(q[3])*sin(q[5])*cos(q[2]))*cos(q[6]) - (-sin(q[2])*cos(q[4]) - sin(q[4])*cos(q[2])*cos(q[3]))*sin(q[6])) 0
    ]
    return z
end


function inv_kinematics(xd, trial=10, α=1, allowance_error=0.001)
    """逆運動学を解く"""

    q1_min, q1_max = -141, 51
    q2_min, q2_max = -123, 60
    q3_min, q3_max = -173, 173
    q4_min, q4_max = -3, 150
    q5_min, q5_max = -175, 175
    q6_min, q6_max = -90, 120
    q7_min, q7_max = -175, 175
    q_min = [q1_min, q2_min, q3_min, q4_min, q5_min, q6_min, q7_min] * π / 180
    q_max = [q1_max, q2_max, q3_max, q4_max, q5_max, q6_max, q7_max] * π / 180

    Δt = 0.01
    for i in 1:trial
        println(i, "回目の試行")
        global q = (rand((7)) .-0.5) .* π  # ランダムな初期値
        for t in 0:Δt:10
            Δx = xd - origins(q)[:, 11]
            error = norm(Δx)
            if error < allowance_error
                break
            else
                dq = α * pinv(jacobi_GL(q)) * Δx
                global q += dq * Δt
            end
        end
        
        if q_min <= q <= q_max
            return q
        else
            continue
        end
    end
end



xd = [0.3, -0.6, 1]  # 所望のエンドエフェクタ位置
qd = inv_kinematics(xd)  # 逆運動学の解を計算

### plot ###
plot(aspect_ratio=1, xlabel="x", ylabel="y", zlabel="z")
plot!(
    [xd[1]], [xd[2]], [xd[3]],
    marker=:circle, markersize = 10, label="desired ef position"
)


data = origins(qd)  # アームの位置データを用意
plot!(data[1, :], data[2, :], data[3, :], marker=:circle, label="arm")
