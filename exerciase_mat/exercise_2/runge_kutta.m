function [next_x]  = runge_kutta(f,x,t)
global dt;
k1 = f(x,t);
k2 = f(x+k1*dt/2,t+dt/2);
k3 = f(x+k2*dt/2,t+dt/2);
k4 = f(x+k3*dt,t+dt);
next_x = x + (k1 + 2*(k2+k3) + k4)*dt/6;
end