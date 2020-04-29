clc 
clear 
close all

%% Parameter 

% Control variables to (de)activate plots
plot_full = 1;
plot_4th = 1;
plot_est = 1;

T_sample = 0.005;                           % time between two samples in [s]


%% Read data

filename = 'experiment_thrust_changes.log';
%filename = 'experiment_roll_changes.log';

log = read_log(filename);                   % log in Hz for two blades

% Choose interested area of the signal
if (strcmp(filename,'experiment_roll_changes.log'))
    log = log(1300:2400)/2;                 % log in Hz for one blade
else
    log = log(670:1550)/2;
end

% Filtering, smoothing
log_sm = smooth(log, 20);

% Plot
if (plot_full)
    figure('Name', 'plot_full');
    plot(log)
    hold on;
    plot(log_sm)
    grid on;
    xlim([0 inf])
    xlabel('Samples [-]')
    ylabel('frequency [Hz]')
end


%% Step

step = zeros(1, size(log, 1));

% Generate step functions (manually)
if (strcmp(filename,'experiment_roll_changes.log'))
    step(1:40) = 100;
    step(41:221) = 289; 
    step(222:433) = 280;
    step(434:620) = 301;
    step(621:826) = 270;
    step(827:1015) = 310;
else
    step(1:12) = 33;
    step(13:207) = 288; 
    step(208:405) = 314;
    step(406:606) = 268;
    step(607:803) = 341;
end
if (plot_full) 
    plot(step); 
    legend('log', 'log smooth', 'step', 'Location', 'best')
end

%% Extract 4th pt1

if (strcmp(filename,'experiment_roll_changes.log'))
    log_4th = log(829:970);
    step_4th = step(829:970)';    
else    
    log_4th = log(606:760);
    step_4th = step(606:760)';
end
t = (0:T_sample:(size(log_4th,1)-1)*T_sample)';

if (plot_4th)
    figure('Name', 'ploth_4th');
    plot(t, log_4th); hold on; grid on; 
    plot(t, step_4th)
    xlabel('Time [s]')
    ylabel('frequency [Hz]')
    xlim([0 inf])
end


%% 1. Estimation Option: tfest()
% Using Matlab function for transfer function estimation

data = iddata(log_4th, step_4th, T_sample);

% Estimate 
G = tfest(data, 1, 0);


%% 2. Estimation Option: Least Squares Fit

ydata = log_4th - log_4th(1);   % subtract offset
tdata = (0:T_sample:(size(ydata,1)-1)*T_sample)';

T = optimvar('T', 1);                   % Create optimization variable
K = optimvar('K', 1);

fun = @(T,K) K*(1-exp(-tdata/T));       % Create function to optimize
response = fcn2optimexpr(fun, T, K);    % Convert function to optim expession
obj = sum((response - ydata).^2);       % Define error function

% Create optimization problem
lsqproblem = optimproblem("Objective", obj);

x0.T = 1/15;                            % Set initial point
x0.K = 70;

% Show problem parameters
%showproblem(lsqproblem);

% Solve the problem
[sol,fval] = solve(lsqproblem, x0);

% Response data for plotting
responsedata = evaluate(response, sol);

% Plot data
if (plot_est)
    fig_est = figure('Name', 'plot_est'); 
    plot(tdata, ydata+log_4th(1)); grid on; hold on; 
    xlim([0 inf]); 
    plot(tdata, responsedata+log_4th(1));
    ylim([responsedata(1)+log_4th(1) 350])
    xlabel('Time [s]');
    ylabel('Rotation frequency [Hz]');
    legend('Measured', 'Fitted', 'Location', 'best');
end


%% 3. Estimation  Option: 63.2% of the steady state
% We look at how much time does it take to reach 63.2% of the steady state
% value. Computation pefrormed on the 4th PT1 element of 
% file 'experiment_thrust_changes.log'
% 
% Result: T = 0.0675 s


%% Plot results
display(['Results: '])
display(['Estimation approximating the transfer function with tfest(): ' num2str(1/G.Denominator(2))]);
display(['Estimation by solving optimization problem: ' num2str(sol.T) ' s']);
display(['Estimation by reading off the value at 63.2% of the steady state value: ' num2str(0.0675) ' s'])


%% Save plot_est

set(fig_est, 'Units', 'points');
pos = get(fig_est, 'Position');                        % gives x left, y bottom, width, height
width = pos(3);
height = pos(4);
set(fig_est, 'PaperUnits', 'points');
set(fig_est, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
set(fig_est, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
saveas(fig_est, 'actuator_dynamic_est', 'pdf');                       % Save figure


