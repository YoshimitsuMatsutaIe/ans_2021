using CPUTime
using DifferentialEquations
using Plots

f = @ode_def begin
    dv = 1/m*(-c*v - k*x + u)
    dx = v
    du = -Kp*u + Ki*(xg-x) - Kd/m*(-c*v - k*x + u)
end m c k xg Kp Ki Kd

m = 1.0
c = 1.0
k = 1.0
xg = 0.0
Kp = 3
Ki = 1
Kd = 0.05

p = [m, c, k, xg, Kp, Ki, Kd]

x0 = [1.0, 0.0, Kp*(xg - 1.0)]  # 初期値を設定
tspan = (0.0, 10.0)  # 解く範囲を設定

#prob = ODEProblem(dx , x0 , tspan, xg, Kp, Ki, Kd)
prob = ODEProblem(f ,x0 ,tspan, p)
sol = solve(prob)  # ソルバで解く

# plot
plot(
    sol,
    linewidth=2,
    xaxis="time [sec]",
    yaxis="",
    label=["position" "velocity" "input"]
)
