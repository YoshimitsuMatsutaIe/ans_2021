function exercise_5()
%EXERCISE_5 
%   icareを使って解く 公式doc:https://jp.mathworks.com/help/control/ref/icare.html

%% 状態方程式
A = [
    1.1 2.0 3.0
    0 0.95 1.20
    1.2 0.01 10.5
    ];
B = [
    1.0
    0.0
    0.847
    ];


%% 重み行列
Q = diag([1000 1000 1000]);
R = 1;

%% リカッチ方程式を解く
S = [];
E = [];
G = [];

[P, K, L] = icare(A, B, Q, R, S, E, G);

disp("P = ");
disp(P);

%% 数値シミュレーション
newA = A - B * K;  % 最適レギュレータの新しいA

dt = 0.01;
tend = 5;
n = tend/dt;
t = linspace(0, tend, n);
X0 = [10; 2; 3];  % 初期値
X = zeros(3, n);
for i = 1:n
    X(:, i) = expm(newA * (i-1)*dt) * X0;
end
%disp(X(:, 1:10));
%% プロット
plot(X(1, :))
plot(t, X(1, :), t, X(2, :), t, X(3, :));
legend('x1', 'x2', 'x3')
xlabel('time [s]')
end

