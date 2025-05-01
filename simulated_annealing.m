function simulated_annealing()
%SIMULATED_ANNEALING  Tune a PID controller via Simulated Annealing
%   Usage: call this function in MATLAB R2015 or later.

    clc;
    clear;
    close all;
    rng(5);  % For reproducibility

    % Define the plant
    plant = tf(1, conv([1 0], conv([2 1], [2 1])));

    % Initial PID parameters [Kp, Ki, Kd]
    initial_params = [10, 10, 10];

    % Simulated Annealing parameter variations
    max_iter_values = [150];
    alpha_values    = [0.95];

    % Prepare storage
    combinations     = numel(max_iter_values) * numel(alpha_values);
    best_params_all  = zeros(combinations, 3);
    labels           = cell(combinations, 1);
    settling_times   = zeros(combinations, 1);
    overshoots       = zeros(combinations, 1);
    rise_times       = zeros(combinations, 1);
    counter = 1;

    % Loop over SA settings
    for max_iter = max_iter_values
        for alpha = alpha_values
            fprintf('\n=== Combination: max_iter=%d, alpha=%.2f ===\n', max_iter, alpha);

            % SA temperatures
            T_init = 100;
            T_min  = 0.1;
            T = T_init;

            % Initialize
            current_params = initial_params;
            best_params    = current_params;
            current_cost   = pid_cost(current_params, plant, 10);
            best_cost      = current_cost;
            iteration      = 0;

            % Simulated Annealing loop
            while T > T_min
                for i = 1:max_iter
                    iteration = iteration + 1;
                    % Neighbor generation
                    new_params = current_params + randn(1,3);
                    new_params = max(new_params, [0.01,0.01,0.01]);

                    % Cost evaluation
                    new_cost = pid_cost(new_params, plant, 10);

                    % Acceptance
                    delta = new_cost - current_cost;
                    if delta < 0 || rand() < exp(-delta/T)
                        current_params = new_params;
                        current_cost   = new_cost;
                        if new_cost < best_cost
                            best_cost   = new_cost;
                            best_params = new_params;
                        end
                    end

                    fprintf('Iter %4d | T=%.2f | Cost=%.4f | Cur=[%.2f,%.2f,%.2f] | Best=[%.2f,%.2f,%.2f]\n', ...
                        iteration, T, current_cost, current_params, best_params);
                end
                T = alpha * T;
            end

            % Store results
            best_params_all(counter,:) = best_params;
            labels{counter}            = sprintf('max_iter=%d, alpha=%.2f', max_iter, alpha);

            % Performance metrics
            C   = pid(best_params(1), best_params(2), best_params(3));
            sys = feedback(C*plant,1);
            info = stepinfo(sys, 'SettlingTimeThreshold', 0.02);
            settling_times(counter) = info.SettlingTime;
            overshoots(counter)     = info.Overshoot;
            rise_times(counter)     = info.RiseTime;

            counter = counter + 1;
        end
    end

    % Plotting results using subplot instead of tiledlayout
    plot_results(plant, initial_params, best_params_all, labels, settling_times, overshoots, rise_times);
end

function J = pid_cost(params, plant, t_max)
%PID_COST   Cost function = Integral of Absolute Error (IAE)
    C = pid(params(1), params(2), params(3));
    sys_cl = feedback(C * plant, 1);
    t = 0:0.01:t_max;
    y = step(sys_cl, t);
    e = 1 - y;
    J = trapz(t, abs(e));
end

function plot_results(plant, initial_params, best_params_all, labels, st, os, rt)
%PLOT_RESULTS   Create figures for step responses and metrics
    combinations = numel(labels);
    t = 0:0.01:25;

    % Figure 1: Step Responses
    figure('Name','Step Responses','Position',[100 100 1200 800]);
    for i = 1:combinations
        subplot(3,3,i);
        C = pid(best_params_all(i,1), best_params_all(i,2), best_params_all(i,3));
        sys = feedback(C*plant,1);
        plot(t, step(sys,t),'LineWidth',1.5); grid on;
        title(labels{i},'FontSize',10);
        xlabel('Time (s)','FontSize',8);
        ylabel('Output','FontSize',8);
        text(2,0.2,sprintf('Kp=%.2f,Ki=%.2f,Kd=%.2f',best_params_all(i,:)),'FontSize',8);
    end
    % Overall title (if suptitle exists)
    if exist('suptitle','file')
        suptitle('Step Responses for Different Settings');
    end

    % Figure 2: Performance Metrics
    figure('Name','Performance Metrics','Position',[100 100 1200 400]);
    % Settling Time
    subplot(1,3,1);
    bar(st);
    set(gca,'XTick',1:combinations,'XTickLabel',labels,'XTickLabelRotation',45);
    title('Settling Time'); xlabel('Setting'); ylabel('Time (s)'); grid on;
    % Overshoot
    subplot(1,3,2);
    bar(os);
    set(gca,'XTick',1:combinations,'XTickLabel',labels,'XTickLabelRotation',45);
    title('Overshoot (%)'); ylabel('%'); grid on;
    % Rise Time
    subplot(1,3,3);
    bar(rt);
    set(gca,'XTick',1:combinations,'XTickLabel',labels,'XTickLabelRotation',45);
    title('Rise Time'); xlabel('Setting'); ylabel('Time (s)'); grid on;
    % Overall title
    if exist('suptitle','file')
        suptitle('Performance Metrics for Different Settings');
    end
end
