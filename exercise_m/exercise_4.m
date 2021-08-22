function exercise_4()
%EXERCISE_4_1 練習問題4
%   1兼2をやる

    function dX = ode(t, X, m, c, k, xg, Kp, Ki, Kd)
        % 微分方程式
        %disp(X)
        x = X(1);
        v = X(2);
        u = X(3);
        
        dx = v;
        dv = 1/m*(-c*v - k*x + u);
        du = -Kp*u + Ki*(xg-x) - Kd/m*(-c*v - k*x + u);
        
        dX = [
            dx
            dv
            du
            ];
    end

%% パラメータ
m = 1.0;
c = 1.0;
k = 1.0;
xg = 1.0;

n = 30;  % 分割数
Kp_range = [0.0 5.0];
Ki_range = [4.5 5.0];
Kd_range = [0.0 0.05];
Kp = linspace(Kp_range(1), Kp_range(2), n);
Ki = linspace(Ki_range(1), Ki_range(2), n);
Kd = linspace(Kd_range(1), Kd_range(2), n);

n_t = 1000;  % 刻み数
tspan = linspace(0, 10, n_t);
x0 = 0.0;
v0 = 0.0;


%% 数値シミュレーション実行
disp("実行中...");
xs = zeros(n_t, n); 
for i = 1:n
    %disp([Kp(i) Ki(i) Kd(i)]);
    X0 = [x0; v0; Kp(i)*(xg - x0)];
    [t, X] = ode45(@(t,X) ode(t, X, m, c, k, xg, Kp(i), Ki(i), Kd(i)), tspan, X0);
    xs(:, i) = X(:, 1);
end


%% アニメ化
disp("アニメーション作成中...");
axis tight manual 
set(gca,'nextplot','replacechildren'); 

v = VideoWriter('exercise_4_mat.avi');
open(v);

plot(t, ones(n_t, 1).* xg);
xlabel('time [sec]', 'FontName','Times New Roman');
ylabel('x', 'FontName','Times New Roman');
xlim([0 10]);
ylim([0.0 xg*1.5]);
%legend('goal');
hold on
p = plot(t, xs(:, 1));
hold off

for i = 2:n
    p.XData = t;
    p.YData = xs(:, i);
    s = "Kp = " + string(round(Kp(i), 1)) + ...
        ", Ki = " + string(round(Ki(i), 1)) + ...
        ", Kd = " + string(round(Kd(i), 1));
    str = text(5, 0.2, s, 'FontName','Times New Roman');
    frame = getframe(gcf);
    writeVideo(v,frame);
    delete(str)
end
close(v);
disp("アニメーション作成完了");
end

