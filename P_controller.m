%------------------------------------------------------------
% Generalized P–gain computation via Bode sampling
% MATLAB R2015 compatible
%------------------------------------------------------------

clear; clc; close all;

%% 1) Specs
Mp       = 0.15;     % 15% overshoot
alpha_c  = 10;       % 10° phase?lead “correction”

%% 2) Plant G(s) = (s+4)/[(s+1)(s+2)(s+6)]   <-- only this line is plant?specific
num = [1 4];
den = conv([1 1], conv([1 2], [1 6]));
G   = tf(num, den);

%% 3) Compute damping ? ? exact ?_PM (deg)
zeta   = -log(Mp)/sqrt(pi^2 + (log(Mp))^2);
phi_PM = atan2(2*zeta, sqrt( sqrt(1+4*zeta^4) - 2*zeta^2 )) * (180/pi);

%% 4) Target loop phase = –180 + ?_PM + ?_c
targetPhase = -180 + phi_PM + alpha_c;

%% 5) Sample Bode at a fine grid
w = logspace(-2, 2, 5000);     % 0.01…100?rad/s
[mag, ph] = bode(G, w);        % mag,ph are 1×1×N arrays
mag = squeeze(mag);
ph  = squeeze(ph);

%% 6) Locate crossover ?c where phase?targetPhase
[~, idx] = min(abs(ph - targetPhase));
wc       = w(idx);             % gain?crossover frequency

%% 7) Compute Kc = 1/|G(j?c)|
Kc = 1/mag(idx);

fprintf('\n--))Computed P–gain: Kc = %.4f\n', Kc);
fprintf('   -> at omega_c = %.3f rad/s (phase = %.2f°)\n\n', wc, ph(idx));

%% 8) Close the loop and plot step responses
T = feedback(Kc*G, 1);

figure;
step(G, 'b--'); hold on;
step(T, 'r-');
grid on;
legend('Plant only','Closed-loop with K_c','Location','Best');
title(sprintf('Step Response (K_c = %.4f)', Kc));
xlabel('Time (s)');
ylabel('Amplitude');
