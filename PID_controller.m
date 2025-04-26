%------------------------------------------------------------
% Manual PID?tuning for 15% overshoot spec
% via frequency?domain (zero placement rule)
% MATLAB R2015
%------------------------------------------------------------

clear; clc; close all;

%% 1) Specs
Mp      = 0.15;     % 15% max overshoot
alpha_c = 10;       % +10° “safety” correction

%% 2) Your plant G(s)  ? only this part is plant?specific
num = [1 4];
den = conv([1 1], conv([1 2], [1 6]));   % (s+1)(s+2)(s+6)
G   = tf(num, den);

%% 3) From Mp ? ? ? exact ?_PM (deg)
zeta   = -log(Mp)/sqrt(pi^2 + (log(Mp))^2);
phi_PM = atan2(2*zeta, sqrt( sqrt(1+4*zeta^4) - 2*zeta^2 )) * (180/pi);

%% 4) Target loop phase for pure?P at ?c
targetPhase = -180 + phi_PM + alpha_c;

%% 5) Find P?only ?c by Bode sampling of ?G(j?)
wgrid = logspace(-1, 1, 5000);       % from 0.1 to 10?rad/s
[~, phG] = bode(G, wgrid);           
phG = squeeze(phG);
[~, idx] = min(abs(phG - targetPhase));
wc = wgrid(idx);                     % gain?crossover freq

%% 6) Choose PID zeros by rule?of?thumb
omega_z1 = wc/10;    Ti = 1/omega_z1;    % zero1 @ wc/10
omega_z2 = 10*wc;    Td = 1/omega_z2;    % zero2 @ 10·wc

%% 7) Compute |C0(jwc)| with C0 = (1 + 1/(Ti s))*(1 + Td s)
magC0 = abs((1i*wc*Ti + 1)/(1i*wc*Ti) * (1i*wc*Td + 1));

%% 8) Solve for Kp so |L(jwc)| = 1  ? Ki, Kd
magGwc = squeeze(bode(G, wc));       % |G(jwc)|
Kp      = 1 / (magGwc * magC0);
Ki      = Kp / Ti;
Kd      = Kp * Td;

%% 9) Display the PID gains & crossover
fprintf('\nPID gains (zero?placement rule):\n');
fprintf('  ?c         = %.3f rad/s\n', wc);
fprintf('  ?z1 = wc/10 = %.3f rad/s (Ti = %.3f s)\n', omega_z1, Ti);
fprintf('  ?z2 = 10·wc = %.3f rad/s (Td = %.3f s)\n', omega_z2, Td);
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f  (integral time Ti = %.3f)\n', Ki, Ti);
fprintf('  Kd = %.4f  (derivative time Td = %.3f)\n\n', Kd, Td);

%% 10) Build the PID controller and plot step responses
Cpid = pid(Kp, Ki, Kd);               % requires Control System Toolbox
Tcl  = feedback(Cpid * G, 1);

figure;
step(G,    'b--'); hold on;
step(Tcl, 'r-');
grid on;
legend('Plant only','PID closed-loop','Location','Best');
title(sprintf('Step Responses with PID (Kp=%.2f, Ki=%.2f, Kd=%.2f)', ...
              Kp, Ki, Kd));
xlabel('Time (s)'); ylabel('Amplitude');
