function example_3()

%% パラメータ
r = 1;  % 半径

%% アニメーションに使うデータを作成
theta = linspace(0, 2*pi);
x = r * cos(theta);
y = r * sin(theta);

%% アニメ化
plot(x,y);
hold on

p = plot(x(1), y(1), 'o','MarkerFaceColor','red');
hold off

axis equal

for i = 2:length(theta)
    p.XData = x(i);
    p.YData = y(i);
    drawnow
end

end