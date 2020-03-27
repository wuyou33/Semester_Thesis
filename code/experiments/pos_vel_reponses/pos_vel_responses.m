clc
clear
close all


%% Import data 

filename = 'exp_1_pos_vel.csv';
[timestamp, posRef_x, posS_x, velRef_x, velS_x] = importfile(filename);


%% Prepare data 

xs = 1000;
xe = 2550;

% TODO delete
velS_x = circshift(velS_x, -15);

posRef_x = posRef_x(xs:xe);
posS_x = posS_x(xs:xe);
velRef_x = velRef_x(xs:xe);
velS_x = velS_x(xs:xe);
timestamp = timestamp(xs:xe)/1000;


%% Plot data 

fig = figure('Name', 'Pos/Vel Responses');

subplot(2, 1, 1);
plot(timestamp, posRef_x); hold on; grid on;
plot(timestamp, posS_x);
xlim([-inf inf])
xlabel('Time $$[s]$$', 'Interpreter', 'latex')
ylabel('$$[m]$$', 'Interpreter', 'latex')
legend('$$x_{ref}$$', '$$x$$', 'Interpreter', 'latex', 'Location', 'southeast')

subplot(2, 1, 2);
plot(timestamp, velRef_x); hold on; grid on;
plot(timestamp, velS_x);
xlim([-inf inf])
xlabel('Time $$[s]$$', 'Interpreter', 'latex')
ylabel('$$[m/s^2]$$', 'Interpreter', 'latex')
legend('$$v_{x,ref}$$', '$$v_x$$', 'Interpreter', 'latex', 'Location', 'southeast')

% Saving plots
set(fig, 'Units', 'points');
pos = get(fig, 'Position');                        % gives x left, y bottom, width, height
width = pos(3);
height = pos(4);
set(fig, 'PaperUnits', 'points');
set(fig, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
set(fig, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
saveas(fig, 'results_responses', 'pdf');                       % Save figure



