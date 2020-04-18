clc 
clear
close all


%% Import data 

[t1, az_r1, az_c1, az_m1] = import('experiment_1.csv');
[t2, az_r2, az_c2, az_m2] = import('experiment_2.csv');
[t3, az_r3, az_c3, az_m3] = import('experiment_3.csv');
[t4, az_r4, az_c4, az_m4] = import('experiment_4.csv');
[t5, az_r5, az_c5, az_m5] = import('experiment_5.csv');
[t6, az_r6, az_c6, az_m6] = import('experiment_6.csv');
[t7, az_r7, az_c7, az_m7] = import('experiment_7.csv');
[t8, az_r8, az_c8, az_m8] = import('experiment_8.csv');
[t9, az_r9, az_c9, az_m9] = import('experiment_9.csv');



%% Shift every signal for a perfect allignment

val = 10;

xs1 = 214-val;
xe1 = 274+val;
az_m1 = az_m1(xs1:xe1);

xs2 = 185-val;
xe2 = 245+val;
az_m2 = az_m2(xs2:xe2);

xs3 = 180-val;
xe3 = 240+val;
az_m3 = az_m3(xs3:xe3);

xs4 = 333-val;
xe4 = 393+val;
az_m4 = az_m4(xs4:xe4);

xs5 = 200-val;
xe5 = 260+val;
az_m5 = az_m5(xs5:xe5);

xs6 = 164-val;
xe6 = 224+val;
az_m6 = az_m6(xs6:xe6);

xs7 = 187-val;
xe7 = 247+val;
az_m7 = az_m7(xs7:xe7);

xs8 = 211-val;
xe8 = 271+val;
az_m8 = az_m8(xs8:xe8);

xs9 = 233-val;
xe9 = 293+val;
az_m9 = az_m9(xs9:xe9);

t = (t1(1:size(az_m1))/1000)';

% hold on; grid on; %1, 2, 6, 7, 8
% plot(az_m1);
% plot(az_m2);
% plot(az_m3);
% plot(az_m4);
% plot(az_m5);
% plot(az_m6);
% plot(az_m7);
% plot(az_m8);
% plot(az_m9);
% xlim([-inf inf])
% ylim([-5 5])


%% Compute mean, standard deviation and plot

signal_matrix = [az_m1, az_m2, az_m3, az_m4, az_m5, az_m6, az_m7...
                 az_m8, az_m9];          
             

mu = mean(signal_matrix, 2);
sigma = std(signal_matrix')';
curve1 = (mu-sigma)';
curve2 = (mu+sigma)';

% Increase number of samples in a signal by a factor 
factor = 1;

t = interp(t, factor);
mu = interp(mu, factor);
curve1 = interp(curve1, factor);
curve2 = interp(curve2, factor);

% Smooth curves
smooth_span = 2;

mu = smooth(mu, smooth_span);
curve1 = (smooth(curve1, smooth_span))';
curve2 = (smooth(curve2, smooth_span))';

% Plot

fig1 = figure()

t_neu = [t, fliplr(t)];
inBetween = [curve1, fliplr(curve2)];
fill(t_neu, inBetween, [.5 .5 .5], 'linestyle', 'none');
hold on; grid on;
plot(t, mu, 'k');
% patch([t' fliplr(t')], [curve1' fliplr(curve2')], 'g')
xlim([-inf inf]);
ylim([-4.5 4.5]);
xlabel('Time $$[s]$$', 'Interpreter', 'latex')
ylabel('$$\dot{v}_{z,measured}~[m/s^2]$$', 'Interpreter', 'latex')
legend('$$\sigma$$', '$$\mu$$', 'Interpreter', 'latex')
             

% Saving plots
set(fig1, 'Units', 'points');
pos = get(fig1, 'Position');                        % gives x left, y bottom, width, height
width = pos(3);
height = pos(4);
set(fig1, 'PaperUnits', 'points');
set(fig1, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
set(fig1, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
saveas(fig1, 'mean_std', 'pdf');                       % Save figure








