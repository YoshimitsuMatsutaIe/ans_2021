function exercise_8()
%EXERCISE_8 倒立振子の制御シミュレーション
%   PIDとLQRを実装

%% パラメータ
p.M = 5.0;  % 車両質量[kg]
p.m = 1.0;  % 振子質量[kg]
p.L = 1.5;  % 振子長さ[m]
p.l = p.L/2;
p.D = 0.01;  % 車両と地面の動摩擦係数
p.d = 0.01;  % 振子と車両の動摩擦係数
p.g = 9.80665;  % 重力加速度[m/s^2]


%% 数値シミュレーション（PID）

    function u = input_by_PID(X, p)
%         % PIDによる制御（thetaのみ）
        %disp(p);
        theta = X(3);
        omega = X(4);
        error = error + (p.xg - theta)*p.dt;
        u = p.Kp*(p.xg - theta) + p.Ki*(error) + p.Kd*(0 - omega);
        disp(error)
        %u = 0;
    end

% パラメータ
input_p.xg = 0;
input_p.Kp = 150;
input_p.Ki = 2;
input_p.Kd = 15;
input_p.dt = 0.1;
tspan = 0:input_p.dt:10;

% 初期値
x_0 = 0;
v_0 = 0;
theta_0 = pi / 6;
omega_0 = 0;
global error  % グローバル変数使うと遅くなる?
error = input_p.xg - theta_0;
X0 = [x_0; v_0; theta_0; omega_0];

[t, X] = ode45(@(t,X) eom(t, X, p, @input_by_PID,input_p), tspan, X0);

%% グラフ化
% k = mod(X(:, 3), 2*pi);
% plot(t, k)
plot(t, X(:, 1))
end


%% ローカル関数
function dX = eom(t, X, p, input, input_param)

x = X(1);
v = X(2);
theta = X(3);
omega = X(4);

multi = [
    1 0 0 0
    0 0 1 0
    0 p.M+p.m 0 p.m*p.l*cos(theta)
    0 p.m*p.l*cos(theta) 0 4/3*p.m*p.l^2
    ];
offset_x = [
    v
    omega
    p.m*p.l*omega^2*sin(theta)-p.D*v
    p.m*p.g*p.l*sin(theta)-p.d*omega
    ];
offset_u = [
    0
    0
    1
    0
    ];

Fa = inv(multi) * offset_x;
Fb = inv(multi) * offset_u;
u = input(X, input_param);  % 入力を計算
%disp(u);
%u = 0;
dX = Fa + Fb*u;
end