%%----------------------------------------------------------------------
% Generic PID tuning comparison using pidtune (MATLAB R2015 compatible)
% - Designs P, PI, and PID controllers on any SISO plant
% - Plots each step response in its own figure
% - Annotates settling time, overshoot, rise time, and steady-state error
% - Prints gain, phase margin, overshoot, settling time, rise time, and steady-state error
%%----------------------------------------------------------------------

clear; clc; close all;

%% 1) Define your plant G(s) here (example below)
% Replace num/den with your system's numerator and denominator
num = [1 4];
den = conv([1 1], conv([1 2], [1 6]));    % Example: (s+1)(s+2)(s+6)
%G = tf(num, den);
G = tf(1, conv([1 0], conv([2 1], [2 1]))); %integrating process
%G = tf(1, [1 10 20]);
%G = tf(10, conv([1 1], [1 2]));

%% 2) Tune controllers using pidtune (automatically selects wc)
[CP,  infoP ] = pidtune(G, 'P');
[CPI, infoPI] = pidtune(G, 'PI');
[CPID,infoPID] = pidtune(G, 'PID');

%% 3) Build closed-loop systems
TP   = feedback(CP   * G, 1);
TPI  = feedback(CPI  * G, 1);
TPID = feedback(CPID * G, 1);

%% 4) Compute performance metrics
infoStepP   = stepinfo(TP);
Ts_P   = infoStepP.SettlingTime;
Tr_P   = infoStepP.RiseTime;
Mp_P   = infoStepP.Overshoot;
essP    = abs(1 - dcgain(TP));

infoStepPI  = stepinfo(TPI);
Ts_PI  = infoStepPI.SettlingTime;
Tr_PI  = infoStepPI.RiseTime;
Mp_PI  = infoStepPI.Overshoot;
essPI  = abs(1 - dcgain(TPI));

infoStepPID = stepinfo(TPID);
Ts_PID = infoStepPID.SettlingTime;
Tr_PID = infoStepPID.RiseTime;
Mp_PID = infoStepPID.Overshoot;
essPID = abs(1 - dcgain(TPID));

%% 5) Plot P-controller response
figure;
step(TP); grid on;
title('Step Response: P Controller');
annotation('textbox',[0.15 0.6 0.3 0.2],...
    'String',{...
      ['K_p = ' num2str(CP.Kp,'%.3f')],...
      ['PM = ' num2str(infoP.PhaseMargin,'%.1f') '°'],...
      ['T_s = ' num2str(Ts_P,'%.3f') ' s'],...
      ['T_r = ' num2str(Tr_P,'%.3f') ' s'],...
      ['M_p = ' num2str(Mp_P,'%.1f') '%'],...
      ['e_{ss} = ' num2str(essP,'%.3f')]},...
    'FitBoxToText','on');

%% 6) Plot PI-controller response
figure;
step(TPI); grid on;
title('Step Response: PI Controller');
annotation('textbox',[0.15 0.6 0.3 0.2],...
    'String',{...
      ['K_p = ' num2str(CPI.Kp,'%.3f')],...
      ['K_i = ' num2str(CPI.Ki,'%.3f')],...
      ['PM = ' num2str(infoPI.PhaseMargin,'%.1f') '°'],...
      ['T_s = ' num2str(Ts_PI,'%.3f') ' s'],...
      ['T_r = ' num2str(Tr_PI,'%.3f') ' s'],...
      ['M_p = ' num2str(Mp_PI,'%.1f') '%'],...
      ['e_{ss} = ' num2str(essPI,'%.3f')]},...
    'FitBoxToText','on');

%% 7) Plot PID-controller response
figure;
step(TPID); grid on;
title('Step Response: PID Controller');
annotation('textbox',[0.15 0.6 0.3 0.2],...
    'String',{...
      ['K_p = ' num2str(CPID.Kp,'%.3f')],...
      ['K_i = ' num2str(CPID.Ki,'%.3f')],...
      ['K_d = ' num2str(CPID.Kd,'%.3f')],...
      ['PM = ' num2str(infoPID.PhaseMargin,'%.1f') '°'],...
      ['T_s = ' num2str(Ts_PID,'%.3f') ' s'],...
      ['T_r = ' num2str(Tr_PID,'%.3f') ' s'],...
      ['M_p = ' num2str(Mp_PID,'%.1f') '%'],...
      ['e_{ss} = ' num2str(essPID,'%.3f')]},...
    'FitBoxToText','on');

%% 8) Display summary in Command Window
fprintf('P:   Kp=%.3f, PM=%.1f°, Ts=%.3f s, Tr=%.3f s, Mp=%.1f%%, ess=%.3f\n', ...
    CP.Kp, infoP.PhaseMargin, Ts_P,   Tr_P,   Mp_P,   essP);
fprintf('PI:  Kp=%.3f, Ki=%.3f, PM=%.1f°, Ts=%.3f s, Tr=%.3f s, Mp=%.1f%%, ess=%.3f\n', ...
    CPI.Kp, CPI.Ki, infoPI.PhaseMargin, Ts_PI,  Tr_PI,  Mp_PI,  essPI);
fprintf('PID: Kp=%.3f, Ki=%.3f, Kd=%.3f, PM=%.1f°, Ts=%.3f s, Tr=%.3f s, Mp=%.1f%%, ess=%.3f\n', ...
    CPID.Kp, CPID.Ki, CPID.Kd, infoPID.PhaseMargin, Ts_PID, Tr_PID, Mp_PID, essPID);
