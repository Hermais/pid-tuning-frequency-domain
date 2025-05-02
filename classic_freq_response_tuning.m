% PID Controller Design for Higher-Order Plant using Frequency Response Methods
% Compatible with MATLAB R2015

clc; clear; close all;

% Step 1: Define the plant (example: 4th order system)
num = 1;
den = conv(conv([1 1], [1 2]), conv([1 3], [1 4]));
G = tf(num, den);
G = tf(1, conv([1 0], conv([2 1], [2 1])));

% Step 2: Design Specifications
PM_desired = 50;        % Desired Phase Margin (degrees)
omega_c = 0.5;          % Initial crossover frequency guess (rad/s)

% Step 3: Get plant response at chosen frequency
[mag_G, phase_G] = bode(G, omega_c);
phase_G = phase_G - 360*(phase_G > 180);  % Phase unwrapping

% Calculate required phase lead
phi_lead = PM_desired - (180 + phase_G);

if phi_lead <= 0
    error('Phase lead <= 0: Increase omega_c or reduce PM_desired');
end

% Step 4: Calculate PD parameters
Kp = cosd(phi_lead) / mag_G;
Kd = (Kp * tand(phi_lead)) / omega_c;

% Step 5: Add integral action
Ki = Kp * omega_c / 10;

% Step 6: Create PID controller with derivative filter
s = tf('s');
PID = Kp + Ki/s + (Kd*s)/(1 + s/(10*omega_c));

% Create open-loop system
L = PID * G;

% Step 7: Check stability margins
[Gm, Pm, Wcg, Wcp] = margin(L);

% Step 8: Create closed-loop system
T = feedback(L, 1);

% Step 9: Simulate and calculate metrics
figure;
step(T);
title('Closed-Loop Step Response');

info = stepinfo(T);
t = 0:0.01:20;
[y, t] = step(T, t);

% Calculate overshoot
[peak, idx] = max(y);
overshoot = 100*(peak - y(end))/y(end);

% Display results
disp('===== Design Results =====');
fprintf('Desired Phase Margin: %.1f°\n', PM_desired);
fprintf('Actual Phase Margin: %.1f°\n', Pm);
fprintf('Crossover Frequency: %.2f rad/s\n\n', Wcp);

disp('PID Gains:');
fprintf('Kp = %.2f\n', Kp);
fprintf('Ki = %.2f\n', Ki);
fprintf('Kd = %.2f\n\n', Kd);

disp('Performance Metrics:');
fprintf('Rise Time: %.2f sec\n', info.RiseTime);
fprintf('Settling Time (2%%): %.2f sec\n', info.SettlingTime);
fprintf('Overshoot: %.1f%% \n', overshoot);