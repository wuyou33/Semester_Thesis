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


%% Add noise

k = 0.5;

az_measured = az_measured(xs:xe);
t = t(xs:xe);

% 1
noise1 = rand(size(az_measured, 1), 1);
noise1 = noise1 - mean(noise1);
az_measured_noise1 = az_measured + k*noise1;

% 2
noise2 = rand(size(az_measured, 1), 1);
noise2 = noise2 - mean(noise2);
az_measured_noise2 = az_measured + k*noise2;

% 3
noise3 = rand(size(az_measured, 1), 1);
noise3 = noise3 - mean(noise3);
az_measured_noise3 = az_measured + k*noise3;

% 4
noise4 = rand(size(az_measured, 1), 1);
noise4 = noise4 - mean(noise4);
az_measured_noise4 = az_measured + k*noise4;

% 5
noise5 = rand(size(az_measured, 1), 1);
noise5 = noise5 - mean(noise5);
az_measured_noise5 = az_measured + k*noise5;


%% Plot data

% fig1 = figure('Name', 'fig1');
% %plot(t(xs:xe), az_reference(xs:xe));
% %plot(t(xs:xe), az_computed(xs:xe)); 
% plot(t(xs:xe), az_measured(xs:xe)); hold on; grid on;
% plot(t(xs:xe), az_measured_noise1(xs:xe));
% plot(t(xs:xe), az_measured_noise2(xs:xe));
% plot(t(xs:xe), az_measured_noise3(xs:xe));
% plot(t(xs:xe), az_measured_noise4(xs:xe));
% plot(t(xs:xe), az_measured_noise5(xs:xe));
% xlim([-inf inf])
% ylim([-3.5 3.5])


A = [az_measured, az_measured_noise1, az_measured_noise2, az_measured_noise3, az_measured_noise4, az_measured_noise5];
mu = mean(A, 2);
st = std(A'); st = st';

figure()
smu = smooth(mu);
smu_minus = smooth(mu-st);
smu_plus = smooth(mu+st);
plot(t, smu); hold on; grid on;
plot(t, smu_minus);
plot(t, smu_plus)
xlim([-inf inf])
ylim([-3.5 3.5])

t2 = [t, fliplr(t)];
inBetween = [smu_plus, fliplr(smu_minus)];
fill(t2, inBetween, 'g');



% numplots = size(i10f.setposn,2);
% meanposni = mean(i10f.setposn,2);
% stdacceli = std(repmat(meanposni,1,numplots)-i10f.setposn,0,2);
% set_t = (0:length(meanposni)-1)/512;
% 
% fill([set_t fliplr(set_t)], [(meanposni+stdacceli).' fliplr(meanposni.'-stdacceli.')],[.5 .5 .5],'linestyle', 'none')
% alpha(0.3)




