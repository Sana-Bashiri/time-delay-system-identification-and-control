clc;
clear;
close all;

% Given system: 1000*e^(-2.5s) / (s^4 + 126 s^3 + 2725 s^2 + 12600 s + 1000)
num = 1000;
den = [1 126 2725 12600 1000];
Gs = tf(num, den, 'InputDelay', 2.5);
tEnd = 60;
t = linspace(0, tEnd, 6001);     
y = step(Gs, t);
K = y(end);                   
h  = mean(diff(t));
dy = gradient(y, h);
[~, idx] = max(dy);
A = [t(idx-1:idx+1).' ones(3,1)];
b = A \ y(idx-1:idx+1);
m = b(1);       
c = b(2);        

% L from tangent intercept with y=0  (y = m t + c)
L = -c/m;

% (Optional) T as well (often needed for 3-parameter model)
tK = (K - c)/m;   % time where tangent hits y=K
T  = tK - L;

% Tangent line for plotting
tv = linspace(0, tEnd, 400).';
f  = m*tv + c;

% Plot (blue response + red tangent + max slope point)
figure
plot(t, y, 'b', 'LineWidth', 2); hold on
plot(tv, f, 'r', 'LineWidth', 2)
plot(t(idx), y(idx), 'ro', 'MarkerSize', 8, 'LineWidth', 2)   % max slope point
plot(L, 0, 'ks', 'MarkerSize', 8, 'LineWidth', 2)             % L at y=0
yline(K, ':');                                                % K level
grid on
xlabel('Time (s)'); ylabel('Output');
legend('Step response','Tangent line','Max slope point','L (tangent hits y=0)','K level', ...
       'Location','southeast');
title('3rd Order');

% Print K and L and T
fprintf('3rd Order:\n');
fprintf('K = %.6f\n', K);
fprintf('L = %.6f s\n', L);
fprintf('T = %.6f s\n', T);
