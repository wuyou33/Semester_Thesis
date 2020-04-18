clc 
close all

%% Import

% Outer loop
xi_ref = out.xi_ref;
xi = out.xi;

dxi_ref = out.dxi_ref;
dxi = out.dxi;

ddxi_ref = out.ddxi_ref;
ddxi = out.ddxi;


% Inner loop
eta_ref = out.eta_ref;
eta = out.eta;

omega_ref = out.omega_ref;
omega = out.omega;

domega_ref = out.domega_ref;
domega = out.domega;

t = out.tout;


%% Plot response - outer loop - xi

% % Plotting
% fig_outer = figure('Name', 'Outer loop response xi');
% for i = 1:3
%     subplot(3, 1, i)
%     plot(t, xi_ref(i, :), 'LineWidth', 1);
%     hold on; grid on;
%     plot(t, xi(i, :), 'LineWidth', 1);
%     xlim([0 inf]);
%     if(i==1)
%         xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
%         ylabel('$$x \ [m]$$', 'Interpreter', 'latex');
%     elseif (i==2)
%         xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
%         ylabel('$$y \ [m]$$', 'Interpreter', 'latex');
%     else
%         xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
%         ylabel('$$z \ [m]$$', 'Interpreter', 'latex');
%     end
%     legend('reference', 'state')
% end
% 
% % Saving
% set(fig_outer, 'Units', 'points');
% pos = get(fig_outer, 'Position');                        % gives x left, y bottom, width, height
% width = pos(3);
% height = pos(4);
% set(fig_outer, 'PaperUnits', 'points');
% set(fig_outer, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
% set(fig_outer, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
% saveas(fig_outer, 'plots/sim_outer', 'pdf');                       % Save figure


%% Plot response - outer loop xi, dxi, ddxi
 
% fig_outer_all = figure('Name', 'Outer loop response xi, dxi, ddxi');
% 
% % Plotting xi
% subplot(3, 1, 1)
% plot(t, xi_ref(1, :), 'LineWidth', 1);
% hold on; grid on;
% plot(t, xi(1, :), 'LineWidth', 1);
% ylim([-0.5 6])
% xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
% ylabel('$$x \ [m]$$', 'Interpreter', 'latex');
% legend('ref', 'state', 'Location', 'best')
% 
% % Plotting dxi
% subplot(3, 1, 2)
% plot(t, dxi_ref(1, :), 'LineWidth', 1);
% hold on; grid on;
% plot(t, dxi(1, :), 'LineWidth', 1);
% ylim([-3 3])
% xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
% ylabel('$$\dot{x} \ [m/s]$$', 'Interpreter', 'latex');
% legend('ref', 'state', 'Location', 'best')
% 
% % Plotting ddxi
% subplot(3, 1, 3)
% plot(t, ddxi_ref(1, :), 'LineWidth', 1);
% hold on; grid on;
% plot(t, ddxi(1, :), 'LineWidth', 1);
% ylim([-5 5])
% xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
% ylabel('$$\ddot{x} \ [m/s^2]$$', 'Interpreter', 'latex');
% legend('ref', 'state', 'Location', 'best')
% 
% % Saving
% set(fig_outer_all, 'Units', 'points');
% pos = get(fig_outer_all, 'Position');                        % gives x left, y bottom, width, height
% width = pos(3);
% height = pos(4);
% set(fig_outer_all, 'PaperUnits', 'points');
% set(fig_outer_all, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
% set(fig_outer_all, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
% saveas(fig_outer_all, 'plots/sim_outer_all', 'pdf');                       % Save figure


%% Plot response - inner loop eta, omega, domega

% fig_inner_all = figure('Name', 'Inner loop response eta, omega, domega');
% 
% % Plotting eta
% subplot(3, 1, 1)
% plot(t, eta_ref(:, 2), 'LineWidth', 0.8);
% hold on; grid on;
% plot(t, eta(2, :), 'LineWidth', 0.8);
% ylim([-0.5 0.5])
% xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
% ylabel('$$\Theta \ [rad]$$', 'Interpreter', 'latex');
% %legend('ref', 'state', 'Location', 'best')
% 
% % Plotting omega
% subplot(3, 1, 2)
% plot(t, omega_ref(2, :), 'LineWidth', 0.8);
% hold on; grid on;
% plot(t, omega(2, :), 'LineWidth', 0.8);
% ylim([-2 2])
% xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
% ylabel('$$\dot{\Theta} \ [rad/s]$$', 'Interpreter', 'latex');
% %legend('ref', 'state', 'Location', 'best')
% 
% % Plotting domega
% subplot(3, 1, 3)
% plot(t, domega_ref(2, :), 'LineWidth', 0.8);
% hold on; grid on;
% plot(t, domega(2, :), 'LineWidth', 0.8);
% ylim([-16 19])
% xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
% ylabel('$$\ddot{\Theta} \ [rad/s^2]$$', 'Interpreter', 'latex');
% %legend('ref', 'state', 'Location', 'best')
% 
% % Saving
% set(fig_inner_all, 'Units', 'points');
% pos = get(fig_inner_all, 'Position');                        % gives x left, y bottom, width, height
% width = pos(3);
% height = pos(4);
% set(fig_inner_all, 'PaperUnits', 'points');
% set(fig_inner_all, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
% set(fig_inner_all, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
% saveas(fig_inner_all, 'plots/sim_inner_all', 'pdf');                       % Save figure



%% Plot response - G2 influence

xi_nog2 = out.xi_nog2;

% Plotting
fig_g2 = figure('Name', 'G2 influence');
plot(t, xi_ref(1, :), 'LineWidth', 1);
hold on; grid on;
plot(t, xi(1, :), '-o', 'MarkerIndices', 1:1000:length(xi(1, :)), 'LineWidth', 1);
plot(t+0.01, xi_nog2(1, :), '-x', 'MarkerIndices', 1:1100:length(xi_nog2(1, :)), 'LineWidth', 1)
xlabel('Time $$[s]$$', 'Interpreter', 'latex');   
ylabel('$$x \ [m]$$', 'Interpreter', 'latex');
xlim([0 inf])
legend('reference', 'response with $$G_2$$', 'response without $$G_2$$', 'Interpreter', 'latex', 'Location', 'best')
set(0,'DefaultLegendAutoUpdate','off')

% Draw circle
a=1.5; % horizontal radius
b=0.25; % vertical radius
x0=12.2; % x0,y0 ellipse centre coordinates
y0=4.8;
time=-pi:0.01:pi;
x=x0+a*cos(time);
y=y0+b*sin(time);
plot(x, y, 'k')

% Create box
axes('position',[0.35 0.18 0.15 0.25])
box on;                                               % put box around new pair of axes
indexOfInterest = (t > 11.40) & (t < 11.60);                % range 
plot(t(indexOfInterest), xi(1, indexOfInterest), 'Color', '#FF0000', 'LineWidth', 1); hold on; grid on;
plot(t(indexOfInterest)+0.01, xi_nog2(1, indexOfInterest), 'Color', '#EDB120', 'LineWidth', 1)
xlim([-inf inf])
ylim([-inf inf])
% Draw arrow
X = [0.33 0.37];
Y = [0.75   0.44];
annotation('arrow',X,Y);

% Saving
set(fig_g2, 'Units', 'points');
pos = get(fig_g2, 'Position');                        % gives x left, y bottom, width, height
width = pos(3);
height = pos(4);
set(fig_g2, 'PaperUnits', 'points');
set(fig_g2, 'PaperPosition', [0 0 width height]);     % Position plot at left hand corner with width 5 and height 5.
set(fig_g2, 'PaperSize', [width height]);             % Set the paper to have width 5 and height 5.
saveas(gcf, 'plots/g2', 'pdf');                       % Save figure
