% manual_pid_tuning.m
% Manual PID tuning via Bode plot (MATLAB R2015 syntax)
% Does not use pidtune. Computes Kp, Ti, Td analytically.

clear; clc; close all;

%----- 1. Define the plant G(s) -----
% Example: second-order system with gain K and poles at a and b
K = 1;
a = 1; b = 2;
G = tf(K, [1 a+b a*b]);    % e.g. G(s) = K / (s^2 + (a+b)s + ab)

%----- 2. Design specifications -----
PM_desired = 50;            % desired phase margin in degrees
wc_desired = 5;             % desired crossover frequency in rad/s

%----- 3. Compute magnitude and phase at wc_desired -----
[mag, phase] = bode(G, wc_desired);
mag = squeeze(mag);         % magnitude (linear)
phase = squeeze(phase);     % phase in degrees

% Compute required Kp such that |Kp*G(jwc)| = 1
Kp = 1/mag;

%----- 4. Compute integral and derivative time constants -----
Ti = 10/wc_desired;        % integrator zero a decade below crossover
alpha = 0.2;                % derivative zero placement factor
Td = alpha/wc_desired;      % derivative zero just below wc
N = 10;                     % filter coefficient for derivative roll-off

%----- 5. Build PID controller C(s) -----
s = tf('s');
C_I = 1 + 1/(Ti*s);
C_D = (Td*s) / (Td/N*s + 1);
C = Kp * C_I * C_D;

%----- 6. Plot Bode: plant vs. compensated open-loop -----
figure;
bode(G, 'b', C*G, 'r--', {1e-1 1e2});
grid on;
legend('Plant G(s)', 'Compensated L(s)=C(s)G(s)');
title('Bode Plot: Before and After PID');

%----- 7. Closed-loop responses -----
CL_no = feedback(G, 1);     % without controller
CL = feedback(C*G, 1);      % with PID controller

% Step response comparison
figure;
step(CL_no, 'b', CL, 'r--');
grid on;
legend('Open-Loop Feedback (G)', 'PID Compensated');
title('Closed-Loop Step Response: Without vs. With PID');

%----- 8. Performance metrics -----
info = stepinfo(CL);
Ts = info.SettlingTime;
Mp = info.Overshoot;

% Steady-state error for unit step
ess = abs(1 - dcgain(CL));

% Display results
fprintf('PID parameters:\n');
fprintf('  Kp  = %.3f\n', Kp);
fprintf('  Ti  = %.3f s\n', Ti);
fprintf('  Td  = %.3f s\n', Td);
fprintf('\nClosed-loop performance with PID:\n');
fprintf('  Settling time Ts = %.3f s\n', Ts);
fprintf('  Percent overshoot Mp = %.2f%%\n', Mp);
fprintf('  Steady-state error ess = %.3f\n', ess);
