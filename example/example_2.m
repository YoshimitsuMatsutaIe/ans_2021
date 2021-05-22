function example_2()
% 例題2：微分方程式
%   以下の状態方程式を数値的に解いてください．また結果をグラフ化してください．

%   dxdt = a * x

%% 解く
    function dx = myODE(t, x)
        % 微分方程式
        a = 1;
        dx = a * x;
    end

tspan = [0.0, 2.0];  % 時間
x0 = 1.0;  % 初期値

[t, y] = ode45(@myODE, tspan, x0);  % 解く

%% グラフ化
plot(t, y);
xlabel('time t');
ylabel('x');
legend('soltion');
grid on
end