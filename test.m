%------------------------------------------------------------
% Semi-smart P, PI, and PID tuning via manual frequency-domain methods
% - Meets maximum overshoot and (rough) settling-time specs
% - Overrides P-controller overshoot if steady-state ? 1
% - Skips any controller that cannot satisfy both specs
% - MATLAB R2015 compatible
%------------------------------------------------------------

clear; clc; close all;

%% 1) User Specs and Plant Definition
Mp_req   = 0.50;      % Desired max overshoot (fraction, e.g., 0.05 for 5%)
Ts_req   = 8;         % Desired max settling time (s)
alpha_c  = 10;        % Phase-lead correction (degrees)

% Define plant G(s) = (s+4)/[(s+1)(s+2)(s+6)]
num = [1 4];
den = conv([1 1], conv([1 2], [1 6]));
G   = tf(num, den);

%% 2) Compute damping ratio and exact phase margin for overshoot spec
zeta    = -log(Mp_req) / sqrt(pi^2 + (log(Mp_req))^2);
phi_PM  = atan2(2*zeta, sqrt(sqrt(1+4*zeta^4) - 2*zeta^2)) * (180/pi);

disp(sprintf('Design specs: MP <= %.1f%%, TS <= %.3f s (zeta=%.3f, phi_PM=%.1f°)', ...
    Mp_req*100, Ts_req, zeta, phi_PM));

% Determine target phase for crossover
targetPhase = -180 + phi_PM + alpha_c;

% Frequency grid to search for crossover
wgrid = logspace(-2, 2, 5000);
[~, ph] = bode(G, wgrid);
ph = squeeze(ph);
[~, idx] = min(abs(ph - targetPhase));
wc = wgrid(idx);
magGwc = abs(bode(G, wc));  % magnitude at chosen crossover

disp(sprintf('Computed crossover frequency: wc = %.3f rad/s', wc));

%% 3) Loop over Controllers
controllers = {'P','PI','PID'};
figCount = 0;
for i = 1:length(controllers)
    switch controllers{i}
        case 'P'
            % P controller
            Kp  = 1 / magGwc;
            C   = Kp;
            ctrlStr = sprintf('P (Kp=%.3f)', Kp);
        case 'PI'
            % PI: zero at wc/10
            Ti   = 10 / wc;
            Kp   = 1 / (magGwc * abs((1i*wc*Ti + 1)/(1i*wc*Ti)));
            Ki   = Kp / Ti;
            C    = tf([Kp*Ti, Kp],[Ti, 0]);
            ctrlStr = sprintf('PI (Kp=%.3f, Ki=%.3f)', Kp, Ki);
        case 'PID'
            % PID: zeros at wc/10 & 10*wc
            Ti   = 10 / wc;
            Td   = 1 / (10*wc);
            magC0 = abs((1i*wc*Ti+1)/(1i*wc*Ti) * (1i*wc*Td+1));
            Kp   = 1 / (magGwc * magC0);
            Ki   = Kp / Ti;
            Kd   = Kp * Td;
            C    = pid(Kp, Ki, Kd);
            ctrlStr = sprintf('PID (Kp=%.3f, Ki=%.3f, Kd=%.3f)', Kp, Ki, Kd);
    end
    
    % Loop-transfer and Closed-loop
    L  = C * G;
    T  = feedback(L, 1);
    
    % Compute standard metrics
    info = stepinfo(T);
    Ts   = info.SettlingTime;
    Mp   = info.Overshoot;
    ess  = abs(1 - dcgain(T));
    [~, PM, ~, ~] = margin(L);
    
    % Override P-controller overshoot if steady-state ? 1
    if strcmp(controllers{i}, 'P')
        [y, t] = step(T);
        y_max = max(y);
        % Overshoot relative to unity
        Mp = max((y_max - 1)*100, 0);
    end
    
    % Check against specs
    if (Mp <= Mp_req*100) && (Ts <= Ts_req)
        figCount = figCount + 1;
        figure(figCount);
        step(T, 'r-', G, 'b--');
        grid on;
        legend([ctrlStr,' Closed-Loop'], 'Plant Only', 'Location', 'Best');
        title(['Controller: ', ctrlStr]);
        annotation('textbox',[0.15 0.6 0.3 0.2], ...
            'String',{ ...
                sprintf('PM = %.1f°', PM), ...
                sprintf('Ts = %.3f s', Ts), ...
                sprintf('Mp = %.1f%%', Mp), ...
                sprintf('e_{ss} = %.3f', ess)}, ...
            'FitBoxToText','on');
        disp(sprintf('%s meets specs: MP=%.1f%% <= %.1f%%, TS=%.3f <= %.3f', ...
            controllers{i}, Mp, Mp_req*100, Ts, Ts_req));
    else
        disp(sprintf('%s-controller cannot meet specs: MP=%.1f%%, TS=%.3f s', ...
            controllers{i}, Mp, Ts));
    end
end

%% 4) Final check
if figCount == 0
    error('No controller could satisfy both the overshoot and settling time requirements.');
end
