function dx = func1(x,t)
global myu
x_1 = x(1);
x_2 = x(2);
    dx = [x_2;myu*(1-x_1^2)*x_2-x_1];
end