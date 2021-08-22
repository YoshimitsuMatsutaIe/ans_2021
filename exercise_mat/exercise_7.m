function exercise_7()
%EXERCISE_7 逆運動学
%   ニュートン法で解く

%% 所望のエンドエフェクタ位置
xd = [0.3 -0.6 1].';


%% パラメータ
% アームの諸元
p.L = 278e-3;
p.h = 64e-3;
p.H = 1104e-3;
p.L0 = 270.35e-3;
p.L1 = 69e-3;
p.L2 = 364.35e-3;
p.L3 = 69e-3;
p.L4 = 374.29e-3;
p.L5 = 10e-3;
p.L6 = 368.3e-3;

% 関節角度の上限下限
q1_min = -141;
q1_max =  51;
q2_min = -123;
q2_max =  60;
q3_min = -173;
q3_max =  173;
q4_min = -3;
q4_max = 150;
q5_min = -175;
q5_max = 175;
q6_min = -90;
q6_max = 120;
q7_min = -175;
q7_max = 175;
q_min = [q1_min q2_min q3_min q4_min q5_min q6_min q7_min].' * pi / 180;
q_max = [q1_max q2_max q3_max q4_max q5_max q6_max q7_max].' * pi / 180;


%% 解を探索
dt = 0.01;
trial = 10;
a = 0.5;

for i = 1:trial
    disp(i);
    disp("回目の試行を実行中...")
    
    q = (rand(7, 1) - 0.5) * pi;  % ランダムな初期値
    for t = 0:dt:10
        dx = xd - o_GL(q, p);  % エンドエフェクタと所望の位置との誤差
        error = norm(dx);
        
        if error < 0.001
            break
        else
            dq = a * pinv(Jacobi_o_GL(q, p)) * dx;
            q = q + dq * dt;
        end
    end
    
    if q_min <= q & q <= q_max
        disp("所望の解を発見!");
        disp("qd = ");
        disp(q);
        break
    else
        disp("失敗");
    end
end


%% 解を図示
% アームのジョイント位置データ作成
origins = os(q, p);
xs = origins(1, :);
ys = origins(2, :);
zs = origins(3, :);

fig = figure;
plot3(xs, ys, zs, 'o-', ...
    xd(1), xd(2), xd(3), '*', 'MarkerSize',10);
legend('joint positions', 'desired end-effector position')
grid on
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
axis equal
end

%% ローカル関数
% ジョイント位置を返す関数
function x = o_Wo(q, p)
x = [
    0
    0
    0
];
end

function x = o_BL(q, p)
x = [
    p.L
    -p.h
    p.H
];
end

function x = o_0(q, p)
x = [
    p.L
    -p.h
    p.H + p.L0
];
end

function x = o_1(q, p)
x = [
    p.L
    -p.h
    p.H + p.L0
];
end

function x = o_2(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) - p.h
    p.H + p.L0
];
end

function x = o_3(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) - 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) - p.h
    p.H + p.L0 - p.L2*sin(q(2))
];
end

function x = o_4(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) - 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) - 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) - p.h
    p.H + p.L0 - p.L2*sin(q(2)) - p.L3*cos(q(2))*cos(q(3))
];
end

function x = o_5(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4)))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) - 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) - 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) + 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) - p.h
    p.H + p.L0 - p.L2*sin(q(2)) - p.L3*cos(q(2))*cos(q(3)) - p.L4*(sin(q(2))*cos(q(4)) + sin(q(4))*cos(q(2))*cos(q(3)))
];
end

function x = o_6(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5)))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) - 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) - 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) + 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5))) - p.h
    p.H + p.L0 - p.L2*sin(q(2)) - p.L3*cos(q(2))*cos(q(3)) - p.L4*(sin(q(2))*cos(q(4)) + sin(q(4))*cos(q(2))*cos(q(3))) + p.L5*((sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(2)))
];
end

function x = o_7(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5)))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) - 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) - 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) + 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5))) - p.h
    p.H + p.L0 - p.L2*sin(q(2)) - p.L3*cos(q(2))*cos(q(3)) - p.L4*(sin(q(2))*cos(q(4)) + sin(q(4))*cos(q(2))*cos(q(3))) + p.L5*((sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(2)))
];
end

function x = o_GL(q, p)
x = [
    p.L + 0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5))) + p.L6*(0.707106781186548*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5)))*sin(q(6)) + 0.707106781186548*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5)))*sin(q(6)) + 0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) + cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(6)) + 0.707106781186548*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) + sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(6)))
    0.707106781186548*p.L1*sin(q(1)) - 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) - 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) - 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1))) + 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5))) + p.L6*(-0.707106781186548*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5)))*sin(q(6)) + 0.707106781186548*(((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5)))*sin(q(6)) - 0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) + cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(6)) + 0.707106781186548*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) + sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(6))) - p.h
    p.H + p.L0 - p.L2*sin(q(2)) - p.L3*cos(q(2))*cos(q(3)) - p.L4*(sin(q(2))*cos(q(4)) + sin(q(4))*cos(q(2))*cos(q(3))) + p.L5*((sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(2))) + p.L6*(((sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(2)))*sin(q(6)) + (-sin(q(2))*cos(q(4)) - sin(q(4))*cos(q(2))*cos(q(3)))*cos(q(6)))
];
end

function xs = os(q, p)
xs = [o_Wo(q,p) o_BL(q,p) o_0(q,p) o_1(q,p) o_2(q,p) o_3(q,p) o_4(q,p) o_5(q,p) o_6(q,p) o_7(q,p) o_GL(q,p)];
end

% エンドエフェクタのヤコビアン行列を返す関数
function J = Jacobi_o_GL(q, p)
J = [
    -0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) - 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) + 0.707106781186548*p.L3*(sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1))) - 0.707106781186548*p.L4*((sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) - 0.707106781186548*p.L4*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) + sin(q(1))*cos(q(2))*cos(q(4))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) + 0.707106781186548*p.L5*(((sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(1))*cos(q(3)))*sin(q(5))) + p.L6*((0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5)))*sin(q(6)) + (0.707106781186548*((sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + 0.707106781186548*(-sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(1))*cos(q(3)))*sin(q(5)))*sin(q(6)) + (0.707106781186548*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) + 0.707106781186548*cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(6)) + (0.707106781186548*(sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(4)) - 0.707106781186548*sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(6))) -0.707106781186548*p.L2*sin(q(1))*sin(q(2)) - 0.707106781186548*p.L2*sin(q(2))*cos(q(1)) - 0.707106781186548*p.L3*sin(q(1))*cos(q(2))*cos(q(3)) - 0.707106781186548*p.L3*cos(q(1))*cos(q(2))*cos(q(3)) - 0.707106781186548*p.L4*(sin(q(1))*sin(q(2))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2))*cos(q(3))) - 0.707106781186548*p.L4*(sin(q(2))*cos(q(1))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2))*cos(q(3))) + 0.707106781186548*p.L5*((sin(q(1))*sin(q(2))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(1))*sin(q(3))*sin(q(5))*cos(q(2))) + 0.707106781186548*p.L5*((sin(q(2))*sin(q(4))*cos(q(1)) - cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(1))*cos(q(2))) + p.L6*((0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + 0.707106781186548*sin(q(1))*sin(q(3))*sin(q(5))*cos(q(2)))*sin(q(6)) + (0.707106781186548*(sin(q(2))*sin(q(4))*cos(q(1)) - cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + 0.707106781186548*sin(q(3))*sin(q(5))*cos(q(1))*cos(q(2)))*sin(q(6)) + (-0.707106781186548*sin(q(1))*sin(q(2))*cos(q(4)) - 0.707106781186548*sin(q(1))*sin(q(4))*cos(q(2))*cos(q(3)))*cos(q(6)) + (-0.707106781186548*sin(q(2))*cos(q(1))*cos(q(4)) - 0.707106781186548*sin(q(4))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(6))) 0.707106781186548*p.L3*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1))) + 0.707106781186548*p.L3*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))) - 0.707106781186548*p.L4*(sin(q(1))*cos(q(3)) - sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(4)) - 0.707106781186548*p.L4*(-sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(1))*cos(q(3)))*sin(q(4)) + 0.707106781186548*p.L5*((sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(4))*cos(q(5))) + 0.707106781186548*p.L5*((sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(4))*cos(q(5)) + (sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(5))) + p.L6*((0.707106781186548*(sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(5)) + 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(4))*cos(q(5)))*sin(q(6)) + 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(4))*cos(q(6)) + (0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(4))*cos(q(5)) + 0.707106781186548*(sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(5)))*sin(q(6)) + 0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(4))*cos(q(6))) -0.707106781186548*p.L4*((sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2))) - 0.707106781186548*p.L4*((sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2))) + 0.707106781186548*p.L5*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(5)) + 0.707106781186548*p.L5*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(5)) + p.L6*(0.707106781186548*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))*cos(q(5)) + (0.707106781186548*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - 0.707106781186548*sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(6)) + 0.707106781186548*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))*cos(q(5)) + (0.707106781186548*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - 0.707106781186548*sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(6))) 0.707106781186548*p.L5*(-((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*sin(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(5))) + 0.707106781186548*p.L5*(-((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*sin(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(5))) + p.L6*((-0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*sin(q(5)) + 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(5)))*sin(q(6)) + (-0.707106781186548*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*sin(q(5)) + 0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(5)))*sin(q(6))) p.L6*((0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5)))*cos(q(6)) + (0.707106781186548*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + 0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5)))*cos(q(6)) - (0.707106781186548*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) + 0.707106781186548*cos(q(1))*cos(q(2))*cos(q(4)))*sin(q(6)) - (0.707106781186548*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) + 0.707106781186548*sin(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))) 0
    0.707106781186548*p.L1*sin(q(1)) + 0.707106781186548*p.L1*cos(q(1)) + 0.707106781186548*p.L2*sin(q(1))*cos(q(2)) + 0.707106781186548*p.L2*cos(q(1))*cos(q(2)) + 0.707106781186548*p.L3*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3))) - 0.707106781186548*p.L3*(sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1))) - 0.707106781186548*p.L4*((sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4))) + 0.707106781186548*p.L4*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) + sin(q(1))*cos(q(2))*cos(q(4))) + 0.707106781186548*p.L5*(((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5))) - 0.707106781186548*p.L5*(((sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + (-sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(1))*cos(q(3)))*sin(q(5))) + p.L6*((0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) + 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5)))*sin(q(6)) + (-0.707106781186548*((sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) - 0.707106781186548*(-sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(1))*cos(q(3)))*sin(q(5)))*sin(q(6)) + (0.707106781186548*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) + 0.707106781186548*cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(6)) + (-0.707106781186548*(sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(4)) + 0.707106781186548*sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(6))) -0.707106781186548*p.L2*sin(q(1))*sin(q(2)) + 0.707106781186548*p.L2*sin(q(2))*cos(q(1)) - 0.707106781186548*p.L3*sin(q(1))*cos(q(2))*cos(q(3)) + 0.707106781186548*p.L3*cos(q(1))*cos(q(2))*cos(q(3)) - 0.707106781186548*p.L4*(sin(q(1))*sin(q(2))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2))*cos(q(3))) + 0.707106781186548*p.L4*(sin(q(2))*cos(q(1))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2))*cos(q(3))) + 0.707106781186548*p.L5*((sin(q(1))*sin(q(2))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(1))*sin(q(3))*sin(q(5))*cos(q(2))) - 0.707106781186548*p.L5*((sin(q(2))*sin(q(4))*cos(q(1)) - cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(1))*cos(q(2))) + p.L6*((0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + 0.707106781186548*sin(q(1))*sin(q(3))*sin(q(5))*cos(q(2)))*sin(q(6)) + (-0.707106781186548*(sin(q(2))*sin(q(4))*cos(q(1)) - cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) - 0.707106781186548*sin(q(3))*sin(q(5))*cos(q(1))*cos(q(2)))*sin(q(6)) + (-0.707106781186548*sin(q(1))*sin(q(2))*cos(q(4)) - 0.707106781186548*sin(q(1))*sin(q(4))*cos(q(2))*cos(q(3)))*cos(q(6)) + (0.707106781186548*sin(q(2))*cos(q(1))*cos(q(4)) + 0.707106781186548*sin(q(4))*cos(q(1))*cos(q(2))*cos(q(3)))*cos(q(6))) -0.707106781186548*p.L3*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1))) + 0.707106781186548*p.L3*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3))) + 0.707106781186548*p.L4*(sin(q(1))*cos(q(3)) - sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(4)) - 0.707106781186548*p.L4*(-sin(q(1))*sin(q(2))*sin(q(3)) - cos(q(1))*cos(q(3)))*sin(q(4)) - 0.707106781186548*p.L5*((sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(4))*cos(q(5))) + 0.707106781186548*p.L5*((sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(4))*cos(q(5)) + (sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(5))) + p.L6*((-0.707106781186548*(sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(5)) - 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(4))*cos(q(5)))*sin(q(6)) - 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(4))*cos(q(6)) + (0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(4))*cos(q(5)) + 0.707106781186548*(sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*sin(q(5)))*sin(q(6)) + 0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(4))*cos(q(6))) 0.707106781186548*p.L4*((sin(q(1))*sin(q(3)) + sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) + sin(q(4))*cos(q(1))*cos(q(2))) - 0.707106781186548*p.L4*((sin(q(1))*sin(q(2))*cos(q(3)) - sin(q(3))*cos(q(1)))*cos(q(4)) + sin(q(1))*sin(q(4))*cos(q(2))) - 0.707106781186548*p.L5*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(5)) + 0.707106781186548*p.L5*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(5)) + p.L6*(-0.707106781186548*(-(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - cos(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))*cos(q(5)) + (-0.707106781186548*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) + 0.707106781186548*sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(6)) + 0.707106781186548*(-(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) - sin(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))*cos(q(5)) + (0.707106781186548*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - 0.707106781186548*sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(6))) -0.707106781186548*p.L5*(-((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*sin(q(5)) + (-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(5))) + 0.707106781186548*p.L5*(-((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*sin(q(5)) + (sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(5))) + p.L6*((0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*sin(q(5)) - 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*cos(q(5)))*sin(q(6)) + (-0.707106781186548*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*sin(q(5)) + 0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*cos(q(5)))*sin(q(6))) p.L6*((-0.707106781186548*((-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*cos(q(4)) - sin(q(4))*cos(q(1))*cos(q(2)))*cos(q(5)) - 0.707106781186548*(-sin(q(1))*cos(q(3)) + sin(q(2))*sin(q(3))*cos(q(1)))*sin(q(5)))*cos(q(6)) + (0.707106781186548*((-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*cos(q(4)) - sin(q(1))*sin(q(4))*cos(q(2)))*cos(q(5)) + 0.707106781186548*(sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)))*sin(q(5)))*cos(q(6)) - (-0.707106781186548*(-sin(q(1))*sin(q(3)) - sin(q(2))*cos(q(1))*cos(q(3)))*sin(q(4)) - 0.707106781186548*cos(q(1))*cos(q(2))*cos(q(4)))*sin(q(6)) - (0.707106781186548*(-sin(q(1))*sin(q(2))*cos(q(3)) + sin(q(3))*cos(q(1)))*sin(q(4)) + 0.707106781186548*sin(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))) 0
    0 -p.L2*cos(q(2)) + p.L3*sin(q(2))*cos(q(3)) - p.L4*(-sin(q(2))*sin(q(4))*cos(q(3)) + cos(q(2))*cos(q(4))) + p.L5*((sin(q(2))*cos(q(3))*cos(q(4)) + sin(q(4))*cos(q(2)))*cos(q(5)) - sin(q(2))*sin(q(3))*sin(q(5))) + p.L6*(((sin(q(2))*cos(q(3))*cos(q(4)) + sin(q(4))*cos(q(2)))*cos(q(5)) - sin(q(2))*sin(q(3))*sin(q(5)))*sin(q(6)) + (sin(q(2))*sin(q(4))*cos(q(3)) - cos(q(2))*cos(q(4)))*cos(q(6))) p.L3*sin(q(3))*cos(q(2)) + p.L4*sin(q(3))*sin(q(4))*cos(q(2)) + p.L5*(sin(q(3))*cos(q(2))*cos(q(4))*cos(q(5)) + sin(q(5))*cos(q(2))*cos(q(3))) + p.L6*((sin(q(3))*cos(q(2))*cos(q(4))*cos(q(5)) + sin(q(5))*cos(q(2))*cos(q(3)))*sin(q(6)) + sin(q(3))*sin(q(4))*cos(q(2))*cos(q(6))) -p.L4*(-sin(q(2))*sin(q(4)) + cos(q(2))*cos(q(3))*cos(q(4))) + p.L5*(sin(q(2))*cos(q(4)) + sin(q(4))*cos(q(2))*cos(q(3)))*cos(q(5)) + p.L6*((sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(6)) + (sin(q(2))*cos(q(4)) + sin(q(4))*cos(q(2))*cos(q(3)))*sin(q(6))*cos(q(5))) p.L5*(-(sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*sin(q(5)) + sin(q(3))*cos(q(2))*cos(q(5))) + p.L6*(-(sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*sin(q(5)) + sin(q(3))*cos(q(2))*cos(q(5)))*sin(q(6)) p.L6*(((sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(3))*cos(q(4)))*cos(q(5)) + sin(q(3))*sin(q(5))*cos(q(2)))*cos(q(6)) - (-sin(q(2))*cos(q(4)) - sin(q(4))*cos(q(2))*cos(q(3)))*sin(q(6))) 0
];
end