function main

clc; clearvars;

global Ks Jh m g h Km Kg Jl Rm

Ks = 1.61;
Jh = 0.0021;
m = 0.403;
g = -9.81;
h = 0.06;
Km = 0.00767;
Kg = 70;
Jl = 0.0059;
Rm = 2.6;

global k0 k1 k2 k3

format long g

A = [0 1 0 0;
     0 0 1 0;
     0 0 0 1;
     0 0 0 0];
 
B = [0; 0; 0; 1];

Q = diag([1e6 1e5 1e-2 1e-2]);

R = 1e-3;

K = lqr(A, B, Q, R);

k0 = K(1);
k1 = K(2);
k2 = K(3);
k3 = K(4);

disp(K);
disp(eig(A-B*K));
disp(A-B*K);

[t, y] = ode45(@fcn, [0 2], [12 -8 -1 1]);

figure('color','white');
for i = 1 : 4
    subplot(2, 2, i);
    plot(t, y(:,i));
    grid on;
end

end


function y = hh(x)

y = x(1) + x(2);

end

function u = uu(x)

v = hh(x);

global Ks Jh m g h Km Kg Jl Rm

global k0 k1 k2 k3

u = (-Jh * Jl * Rm * (v * Jl * k0 + Jl * (x(3) + x(4)) * k1 + ...
    (-Ks * x(2) + g * h * m * sin(x(1) + x(2))) * k2 + ...
    (-Ks * x(4) + g * h * m * (x(3) + x(4)) * cos(x(1) + x(2))) * k3) + ...
    (Ks * (-((Jh + Jl) * Ks * Rm * x(2)) + Jl * Kg^2 * Km^2 * x(3)) + g * h * Jh * m * Rm * (Ks * x(2) * cos(x(1) + x(2)) + (Ks + Jl * (x(3) + x(4))^2 - g * h * m * cos(x(1) + x(2))) * sin(x(1) + x(2))))) ...
    / (Jl * Kg * Km * Ks);

end

function dx = fcn(t, y)

u = uu(y);

global Ks Jh m g h Km Kg Jl Rm

f = [y(3);
     y(4);
     Ks / Jh * y(2) - (Km^2 * Kg^2) / (Rm * Jh) * y(3);
     -(Ks / Jh + Ks / Jl) * y(2) + m * g * h / Jl * sin(y(1) + y(2)) + (Km^2 * Kg^2) / (Rm * Jh) * y(3)];
 
ge = [0;
     0;
     (Km * Kg) / (Rm * Jh);
     -(Km * Kg) / (Rm * Jh)];

dx = f + ge * u;

end