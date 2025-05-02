%% Semi-smart P, PI, and PID tuning via manual frequency-domain methods
% - Tries to meet maximum overshoot and (rough) settling-time specs
% - Relies on the notion that 2nd order systems can well approximate higher
%   order systems.
% - Skips any controller that cannot satisfy both specs
% - Integrator-aware PI/PID for plants with built-in integrators
%------------------------------------------------------------
clear; clc; close all;

%% 1) User Specs and Plant Definition
Mp_req   = 0.16;      % Desired max overshoot (fraction, e.g., 0.05 for 5%)
Ts_req   = 400;       % Desired max settling time (s)

% Define plant G(s) here.
%G   = tf([1 4], conv([1 1], conv([1 2], [1 6])));
G = tf(1, conv([1 0], conv([2 1], [2 1])));  % integrating process example
%G = tf(1, [1 10 20]);
%G = tf(10, conv([1 1],  [1 2]));

%% 2) Compute damping ratio and exact phase margin for overshoot spec
zeta    = -log(Mp_req) / sqrt(pi^2 + (log(Mp_req))^2); % From second order systems
phi_PM  = atan2(2*zeta, sqrt(sqrt(1+4*zeta^4) - 2*zeta^2)) * (180/pi); % From second order systems


% Print specs, damping ratio, and phase margin.
fprintf('-->Design specs: MP <= %.1f%%, TS <= %.3f s (zeta=%.3f, phi_PM=%.1f°)\n', ...
    Mp_req*100, Ts_req, zeta, phi_PM);

targetPhase = -180 + phi_PM;
phi_lead = phi_PM - (180 + targetPhase);

%% 2.1) Frequency response of the plant and crossover frequency
wgrid = logspace(-2, 2, 5000);
[~, ph] = bode(G, wgrid); ph = squeeze(ph);
logw    = log10(wgrid);
logw_c  = interp1(ph, logw, targetPhase, 'linear');
wc      = 10^logw_c;
magGwc  = abs(evalfr(G, 1j*wc)); % Evaluates the plant G magnitude at wc in frequency domain.
fprintf('-->Computed crossover frequency: wc = %.3f rad/s\n', wc);

%% 2.5) Detect built-in integrators in G and compute scaling factor
polesG     = pole(G);
tol        = 1e-6;
nInt       = sum(abs(polesG) < tol);
scaleIntTI = 10^nInt;
fprintf('-->Plant has %d integrator pole(s); scaling Ti by %.1f\n', nInt, scaleIntTI);

%% 3) Loop over controllers
controllers = {'P','PI','PID'};
figCount    = 0;
for i = 1:length(controllers)
    switch controllers{i}
      case 'P'
        Kp  = cosd(phi_lead) / magGwc;
        C = Kp;
        ctrlStr = sprintf('-->P (Kp=%.3f)', Kp);

      case 'PI'
        Kp       = cosd(phi_lead) / magGwc;
        Ti_base = 10 / wc;       Ti = Ti_base * scaleIntTI;
        Ki       = Kp / Ti;
        C        = tf([Kp*Ti, Kp], [Ti, 0]);
        ctrlStr  = sprintf('-->PI (Kp=%.3f, Ki=%.3f, Ti=%.3f)', Kp, Ki, Ti);

      case 'PID'
        Kp       = cosd(phi_lead) / magGwc;
        Ti_base = 10 / wc;       Ti = Ti_base * scaleIntTI;
        Ki       = Kp / Ti;
        %Td       = 1 / (10*wc);
        Kd       =( Kp * tand(phi_lead) )/wc;
        C        = pid(Kp, Ki, Kd);
        ctrlStr  = sprintf('-->PID (Kp=%.3f, Ki=%.3f, Kd=%.3f, Ti=%.3f)', ...
                           Kp, Ki, Kd, Ti);
    end

    % Loop-transfer and closed-loop TF
    L  = C * G;
    T  = feedback(L, 1);

    % Compute performance metrics
    info    = stepinfo(T);
    Tr      = info.RiseTime;          % Rise Time (10%-90%)
    Ts      = info.SettlingTime;
    Mp      = info.Overshoot;
    ess     = abs(1 - dcgain(T));
    [Gm, PM, Wgm, Wpm] = margin(L);

    % This clamps the P-controller Mp to zero if response is less than 1.
    if strcmp(controllers{i}, 'P')
      [y, ~] = step(T);
      Mp = max((max(y)-1)*100, 0);
    end

    % Check against specs and plot if OK
    if (Mp <= Mp_req*100) && (Ts <= Ts_req)
      figCount = figCount + 1;

      %% (a) Step response with annotations
      figure(figCount);
      step(T, 'r-', G, 'b--'); grid on;
      legend([ctrlStr,' Closed-Loop'], 'Plant Only', 'Location','Best');
      title(['Controller: ', ctrlStr]);
      annotation('textbox',[0.15 0.6 0.3 0.2], ...
        'String',{...
          sprintf('PM = %.1f°', PM), ...
          sprintf('Ts = %.3f s', Ts), ...
          sprintf('Mp = %.1f%%', Mp), ...
          sprintf('Tr = %.3f s', Tr), ...
          sprintf('e_{ss} = %.3f', ess), ...
          sprintf('omega_{PM} = %.3f rad/s', Wpm), ...
          }, ...
        'FitBoxToText','on');
      fprintf('-->%s meets specs: MP=%.1f%% <= %.1f%%, TS=%.3f <= %.3f, Tr=%.3f s\n', ...
              controllers{i}, Mp, Mp_req*100, Ts, Ts_req, Tr);

    else
      fprintf('-->%s-controller cannot meet specs (Mp=%.1f%%, Ts=%.3f s)\n', ...
              controllers{i}, Mp, Ts);
    end
end

%% (b) Bode plot of the plant for refrence.
      figure; 
      bode(G); 
      grid on;
      title('Bode Plot for Plant G');
      yShift = 0.2 ;  % shift down by 0.2 for annotation
      annotation('textbox',[0.15 yShift 0.3 0.2], ...
        'String',{...
          sprintf('Phase Margin (approximated) = %.1f°', PM), ...
          sprintf('omega_{PM} (approximated) = %.3f rad/s', Wpm)}, ...
        'FitBoxToText','on');


%% 4) Final check
if figCount == 0
  error('-->No controller could satisfy both the overshoot and settling time requirements.');
end