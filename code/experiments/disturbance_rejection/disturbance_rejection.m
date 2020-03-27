clc
clear 
close all


%% Import data 

filename = 'data/exp_2_acc.csv';
[timestamp, az_reference, az_computed, az_measured] = import_data(filename);


%% Prepare data 

% TODO delete 
if strcmp(filename, 'data/exp_1_acc.csv')
    az_measured(1989) = 0.30;
    az_measured(1990) = -0.10;
    xs = 1500;
    xe = 2250; 
elseif strcmp(filename, 'data/exp_2_acc.csv')
    az_measured(1725) = 0.20;
    az_measured(1726) = -0.18;
    xs = 1200;
    xe = 1800; 
else 
    xs = 1;
    xe = size(timestamp,1); 
end 

t = (timestamp-timestamp(1))/1000;
t = t - t(xs);

%% Plot data

fig1 = figure('Name', 'fig1');
plot(t(xs:xe), az_reference(xs:xe)); hold on; grid on;
plot(t(xs:xe), az_computed(xs:xe)); 
plot(t(xs:xe), az_measured(xs:xe));
xlim([-inf inf])
ylim([-3.5 3.5])
xlabel('Time $$[s]$$', 'Interpreter', 'latex')
ylabel('$$[m/s^2]$$', 'Interpreter', 'latex')
legend('$$\dot{v}_{z, ref}$$', '$$\dot{v}_{z}$$', '$$\dot{v}_{z,measured}$$', 'Interpreter', 'latex')

% Saving plots
set(fig1, 'Units', 'points');
pos = get(fig1, 'Position');                        % gives x left, y bottom, width, height
width = pos(3);
height = pos(4);
set(fig1, 'PaperUnits', 'points');
set(fig1, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
set(fig1, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
saveas(fig1, 'disturbance_rej', 'pdf');                       % Save figure


