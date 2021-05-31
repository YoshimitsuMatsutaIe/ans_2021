function example_0()

%% �p�����[�^
start = -5;  % ������';'��t���Ȃ��ƃR�}���h�E�B���h�E�ɏo�͂���Ă��܂�
stop = 10;
step = 0.1;

%% x�̐���쐬
x = start:step:stop;

%% �񎟊֐��̐���쐬
y = x.^2 + x + 1;  % �p��'^'�ipython����**�j�D'.'��t���邱�Ƃŗv�f���̙p��ɂȂ�D

%% ���������Ȃ̂ŃO���t��
plot(x, y);

end