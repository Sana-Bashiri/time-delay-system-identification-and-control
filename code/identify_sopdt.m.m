clc;
clear;
close all;

% Given system
num = 1000;
den = [1 126 2725 12600 1000];
Gs  = tf(num, den, 'InputDelay', 2.5);

% Step response data
tEnd = 60;
t = linspace(0, tEnd, 6001);          % dt = 0.01
y = step(Gs, t);


K0 = y(end);
h  = mean(diff(t));
dy = gradient(y, h);
[~, idx] = max(dy);

A = [t(idx-1:idx+1).' ones(3,1)];
b = A \ y(idx-1:idx+1);
m = b(1);  c = b(2);

L0 = max(0, -c/m);
tK = (K0 - c)/m;
T0 = max(1e-3, tK - L0);

T10 = 0.7*T0;                         % split T into two time constants
T20 = 0.3*T0;

% --- Nonlinear fit for SOPDT parameters: [K, T1, T2, L] ---
% Optimize in log-domain to keep parameters positive
x0 = log([max(K0,1e-6), max(T10,1e-6), max(T20,1e-6), max(L0,1e-6)]);

cost = @(x) sopdt_cost(exp(x), t, y);

opts = optimset('Display','off', 'MaxFunEvals', 2e4, 'MaxIter', 2e4);
xhat = fminsearch(cost, x0, opts);

p = exp(xhat);
K  = p(1);
T1 = p(2);
T2 = p(3);
L  = p(4);

% Sort so T1 >= T2 (optional, just for neatness)
if T2 > T1
    tmp = T1; T1 = T2; T2 = tmp;
end

G4 = tf(K, conv([T1 1],[T2 1]), 'InputDelay', L);
y4 = step(G4, t);

% Plot (original vs SOPDT)
figure
plot(t, y, 'b', 'LineWidth', 2); hold on
plot(t, y4, 'r--', 'LineWidth', 2);
grid on
xlabel('Time (s)'); ylabel('Output');
legend('Original system', '4th Order', 'Location', 'southeast');
title('4th Order');

% Print parameters
fprintf('4th Order:\n');
fprintf('K  = %.6f\n', K);
fprintf('T1 = %.6f s\n', T1);
fprintf('T2 = %.6f s\n', T2);
fprintf('L  = %.6f s\n', L);

% -------- local function --------
function J = sopdt_cost(p, t, y)
    K  = p(1); T1 = p(2); T2 = p(3); L = p(4);

    % Penalize bad values hard (keeps optimizer sane)
    if any(~isfinite(p)) || any(p <= 0) || T1 < 1e-6 || T2 < 1e-6 || L < 1e-6
        J = 1e12; return;
    end

    G = tf(K, conv([T1 1],[T2 1]), 'InputDelay', L);
    ym = step(G, t);

    e = ym - y;
    J = e.'*e;   % SSE
end
