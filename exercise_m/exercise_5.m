function exercise_5()
%EXERCISE_5 
%   icare���g���ĉ��� ����doc:https://jp.mathworks.com/help/control/ref/icare.html

%% ��ԕ�����
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


%% �d�ݍs��
Q = diag([1000 1000 1000]);
R = 1;

%% ���J�b�`������������
S = [];
E = [];
G = [];

[P, K, L] = icare(A, B, Q, R, S, E, G);

disp("P = ");
disp(P);

%% ���l�V�~�����[�V����
newA = A - B * K;  % �œK���M�����[�^�̐V����A

dt = 0.01;
tend = 5;
n = tend/dt;
t = linspace(0, tend, n);
X0 = [10; 2; 3];  % �����l
X = zeros(3, n);
for i = 1:n
    X(:, i) = expm(newA * (i-1)*dt) * X0;
end
%disp(X(:, 1:10));
%% �v���b�g
plot(X(1, :))
plot(t, X(1, :), t, X(2, :), t, X(3, :));
legend('x1', 'x2', 'x3')
xlabel('time [s]')
end

