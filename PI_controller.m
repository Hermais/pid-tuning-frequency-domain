%------------------------------------------------------------
% PI tuning by zero at one decade below wc
% for 15% overshoot spec (manual freq-domain)
% MATLAB R2015
%------------------------------------------------------------

clear; clc; close all;

%% 1) Specs
Mp       = 0.15;    % 15% overshoot
alpha_c  = 10;      % +10° “safety” correction

%% 2) Plant G(s)
num = [1 4];
den = conv([1 1], conv([1 2], [1 6]));
G   = tf(num, den);

%% 3) Find ? ? exact ?_PM (deg)
zeta   = -log(Mp)/sqrt(pi^2 + (log(Mp))^2);
phi_PM = atan2(2*zeta, sqrt( sqrt(1+4*zeta^4) - 2*zeta^2 )) * (180/pi);

%% 4) Target phase for P-only
targetPhase = -180 + phi_PM + alpha_c;

%% 5) Locate ?c by sampling ?G(j?)
wgrid = logspace(-1, 1, 5000);       % 0.1…10?rad/s
[~, phG] = bode(G, wgrid);
phG = squeeze(phG);
[~, idx] = min(abs(phG - targetPhase));
wc = wgrid(idx);                     % P-only crossover

%% 6) Place PI zero at ?z = wc/10  ? Ti = 1/?z
omega_z = wc/10;
Ti      = 1/omega_z;

%% 7) Compute magnitude at ?c and solve for Kp
magGwc  = squeeze(bode(G, wc));
% PI controller (no Kp) has |C0(jwc)| = |(Ti*jwc+1)/(Ti*jwc)|
magC0wc = abs( (1i*wc*Ti + 1) / (1i*wc*Ti) );
Kp      = 1 / (magGwc * magC0wc);
Ki      = Kp / Ti;

%% 8) Display results
fprintf('\nPI controller tuned by zero @ wc/10:\n');
fprintf('  ?c (P-only) = %.3f rad/s\n', wc);
fprintf('  Zero at ?z   = %.3f rad/s  (Ti = %.3f s)\n', omega_z, Ti);
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f\n\n', Ki);

%% 9) Plot step responses
Cpi = tf([Kp*Ti, Kp], [Ti, 0]);      % Kp*(Ti s + 1)/(Ti s)
Tcl = feedback(Cpi * G, 1);

figure;
step(G,    'b--'); hold on;
step(Tcl, 'r-');
grid on;
legend('Plant only','PI closed-loop','Location','Best');
title(sprintf('Step Response with PI (Kp=%.2f, Ki=%.2f)', Kp, Ki));
xlabel('Time (s)'); ylabel('Amplitude');
