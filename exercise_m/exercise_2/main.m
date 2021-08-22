clearvars

global dt myu
t0 = 0; %初期時刻
te = 80; %終端時刻
dt = 0.1; %刻み幅

% 被積分関数:x''-myu(1-x^2)x'+x=0 %
myu = 1.0;
x = [0;1]; %初期値x(0) = [x(0)(初期位置);dx/dt(0)(初速度)] = [x1(0);x2(0)]
t = t0;

result = [t,x.']; %時刻tとxの解を格納する箱の用意．「.'」で転置を表す．

for i = 2:1:(te-t0)/dt+1 %(2行目から格納するため):(個数だから):(+1を忘れない)
    x = runge_kutta(@func1,x,t); %関数に関数を渡すときは関数ハンドルを使う＝"@"を関数の前につける．
    t = t + dt;
    result(i,:) = [t,x.'];
end

figure(1)
hold on;
num_1 = plot(result(:,1),result(:,2),"r");
box on;
grid on;
xlabel('t','FontSize',30,'FontName','Times New Roman')
ylabel('x_1','FontSize',30,'FontName','Times New Roman')

figure(2)
hold on;
num_2 = plot(result(:,1),result(:,3),"b");
box on;
grid on;
xlabel('t','FontSize',30,'FontName','Times New Roman')  
ylabel('x_2','FontSize',30,'FontName','Times New Roman')

figure(3)
hold on;
num_3 = plot(result(:,2),result(:,3),"g");
box on;
grid on;
xlabel('x_1','FontSize',30,'FontName','Times New Roman')
ylabel('x_2','FontSize',30,'FontName','Times New Roman')

