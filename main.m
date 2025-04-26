%------------------------------------------------------------
% Combined P, PI, and PID tuning via manual frequency-domain methods
% - Computes and plots step responses for P, PI, PID controllers
% - Annotates Phase Margin, Settling Time, Overshoot, and Steady-State Error
% - MATLAB R2015 compatible
%------------------------------------------------------------

clear; clc; close all;

%% 1) Specs and Plant
Mp      = 0.05;    % Desired maximum overshoot (15%)
alpha_c = 10;      % Phase-lead correction (degrees)

% Define plant G(s) = (s+4)/[(s+1)(s+2)(s+6)]
num = [1 4];
den = conv([1 1], conv([1 2], [1 6]));
G   = tf(num, den);

%% 2) Compute damping ratio and exact phase margin
zeta   = -log(Mp)/sqrt(pi^2 + (log(Mp))^2);
phi_PM = atan2(2*zeta, sqrt(sqrt(1+4*zeta^4) - 2*zeta^2)) * (180/pi);

%% 3) Determine crossover frequency for P-controller
targetPhase = -180 + phi_PM + alpha_c;
wgrid = logspace(-2, 2, 5000);
[~, ph] = bode(G, wgrid);
ph = squeeze(ph);
[~, idx] = min(abs(ph - targetPhase));
wc = wgrid(idx);

%% 4) P-controller design
Kp_P = 1 / abs(bode(G, wc));    % Kc = 1/|G(jwc)|
L_P  = Kp_P * G;

% Closed-loop transfer and metrics
T_P = feedback(L_P, 1);
infoP = stepinfo(T_P);
Ts_P  = infoP.SettlingTime;
Mp_P  = infoP.Overshoot;
ess_P = abs(1 - dcgain(T_P));
[~, PM_P, ~, ~] = margin(L_P);

% Plot P-controller response
figure(1);
step(T_P, 'r-'); hold on;
step(G,   'b--'); hold off;
grid on;
legend('P Closed-Loop','Plant Only','Location','Best');
title(sprintf('P Controller (Kp=%.3f)', Kp_P));
annotation('textbox',[0.15 0.6 0.3 0.2], ...
    'String',{... 
      sprintf('PM = %.1f°', PM_P), ...
      sprintf('Ts = %.3f s', Ts_P), ...
      sprintf('Mp = %.1f%%', Mp_P), ...
      sprintf('e_{ss} = %.3f', ess_P)}, ...
    'FitBoxToText','on');

%% 5) PI-controller design (zero at wc/10)
omega_z1 = wc/10;
Ti      = 1/omega_z1;

% Compute Kp_PI and Ki
magGwc  = abs(bode(G, wc));
magC0PI = abs((1i*wc*Ti + 1) / (1i*wc*Ti));
Kp_PI   = 1/(magGwc * magC0PI);
Ki_PI   = Kp_PI / Ti;
C_PI    = tf([Kp_PI*Ti, Kp_PI], [Ti, 0]);

L_PI = C_PI * G;
T_PI = feedback(L_PI, 1);
infoPI = stepinfo(T_PI);
Ts_PI  = infoPI.SettlingTime;
Mp_PI  = infoPI.Overshoot;
ess_PI = abs(1 - dcgain(T_PI));
[~, PM_PI, ~, ~] = margin(L_PI);

% Plot PI-controller response
figure(2);
step(T_PI, 'r-'); hold on;
step(G,    'b--'); hold off;
grid on;
legend('PI Closed-Loop','Plant Only','Location','Best');
title(sprintf('PI Controller (Kp=%.3f, Ki=%.3f)', Kp_PI, Ki_PI));
annotation('textbox',[0.15 0.6 0.3 0.2], ...
    'String',{...
      sprintf('PM = %.1f°', PM_PI), ...
      sprintf('Ts = %.3f s', Ts_PI), ...
      sprintf('Mp = %.1f%%', Mp_PI), ...
      sprintf('e_{ss} = %.3f', ess_PI)}, ...
    'FitBoxToText','on');

%% 6) PID-controller design (zeros at wc/10 and 10*wc)
omega_z1 = wc/10;    Ti = 1/omega_z1;
omega_z2 = 10*wc;    Td = 1/omega_z2;

magC0PID = abs((1i*wc*Ti + 1)/(1i*wc*Ti) * (1i*wc*Td + 1));
Kp_PID   = 1/(magGwc * magC0PID);
Ki_PID   = Kp_PID / Ti;
Kd_PID   = Kp_PID * Td;
C_PID    = pid(Kp_PID, Ki_PID, Kd_PID);

L_PID = C_PID * G;
T_PID = feedback(L_PID, 1);
infoPID = stepinfo(T_PID);
Ts_PID  = infoPID.SettlingTime;
Mp_PID  = infoPID.Overshoot;
ess_PID = abs(1 - dcgain(T_PID));
[~, PM_PID, ~, ~] = margin(L_PID);

% Plot PID-controller response
figure(3);
step(T_PID, 'r-'); hold on;
step(G,     'b--'); hold off;
grid on;
legend('PID Closed-Loop','Plant Only','Location','Best');
title(sprintf('PID Controller (Kp=%.3f, Ki=%.3f, Kd=%.3f)', Kp_PID, Ki_PID, Kd_PID));
annotation('textbox',[0.15 0.6 0.3 0.2], ...
    'String',{...
      sprintf('PM = %.1f°', PM_PID), ...
      sprintf('Ts = %.3f s', Ts_PID), ...
      sprintf('Mp = %.1f%%', Mp_PID), ...
      sprintf('e_{ss} = %.3f', ess_PID)}, ...
    'FitBoxToText','on');
