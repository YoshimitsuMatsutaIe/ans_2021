function exercise_4()
%EXERCISE_4_1 ���K���4
%   1��2�����

    function dX = ode(t, X, m, c, k, xg, Kp, Ki, Kd)
        % ����������
        
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

%% �p�����[�^
m = 1.0;
c = 1.0;
k = 1.0;
xg = 1.0;
Kp = 10;
Ki = 1;
Kd = 0.05;

tspan = [0, 10];
X0 = [
    0
    0
    Kp*(xg - 0.0)
    ];

[t, X] = ode45(@(t,X) ode(t, X, m, c, k, xg, Kp, Ki, Kd), tspan, X0);


%% �O���t��
plot(t, X(:,1));

xlabel('time t');
ylabel('x');
legend('soltion');
grid on


end

