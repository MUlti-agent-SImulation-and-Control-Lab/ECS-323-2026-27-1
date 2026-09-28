%% ECS-323 Course Project
% main code run this only properly 
% Run this file using:
% >> main

clear;
clc;
close all;

%% Project parameters
params.g = 9.81;       % gravitational acceleration (m/s^2)
params.x0 = 0.0;       % initial horizontal position (m)
params.z0 = 2.0;       % initial height (m)
params.vx0 = 3.5;      % initial horizontal velocity (m/s)
params.vz0 = 5.0;      % initial vertical velocity (m/s)

params.measurementNoiseStd = 0.06;  % sensor noise (m)
params.numSamples = 80;             % number of simulated measurements
params.randomSeed = 7;              % fixed seed for reproducibility

%% Generate t and M
[t, xTrue, zTrue, xMeasured, zMeasured] = ...
    simulate_trajectory(params);

%% Estimate point from the noise meansurement
[landingX, landingT, fitCoefficients] = ...
    predict_landing_point(t, xMeasured, zMeasured);

%% Displaying result
fprintf('\n============================================\n');
fprintf(' Vision-Guided Dustbin - Milestone 1\n');
fprintf(' Trash Landing Prediction\n');
fprintf('============================================\n');
fprintf('Predicted landing time : %.3f s\n', landingT);
fprintf('Predicted landing x    : %.3f m\n', landingX);
fprintf('============================================\n\n');

%% Plot
figure('Name','Trash Trajectory and Landing Prediction', ...
       'Color','w');

plot(xTrue, zTrue, 'k-', 'LineWidth', 2);
hold on;

plot(xMeasured, zMeasured, 'o', ...
     'MarkerSize', 4);

% Estimated trajectory part 2
tFit = linspace(0, landingT, 200);
zFit = polyval(fitCoefficients, tFit);
xFit = polyval(polyfit(t, xMeasured, 1), tFit);

plot(xFit, zFit, '--', 'LineWidth', 1.8);

plot(landingX, 0, 'p', ...
     'MarkerSize', 13, ...
     'MarkerFaceColor', 'w');

yline(0, ':', 'Ground');

grid on;
xlabel('Horizontal position, x (m)');
ylabel('Height, z (m)');
title('Trash Trajectory and Predicted Landing Point');

legend('True trajectory', ...
       'Noisy measurements', ...
       'Estimated trajectory', ...
       'Predicted landing point', ...
       'Location','best');

xlim([min(xTrue)-0.2, max(xTrue)+0.5]);
ylim([min(-0.1, min(zMeasured)-0.1), max(zTrue)+0.3]);

%% Save result
projectRoot = fileparts(fileparts(mfilename('fullpath')));
resultsDir = fullfile(projectRoot, 'results');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

saveas(gcf, fullfile(resultsDir, 'trajectory_prediction.png'));

fprintf('Result saved to:\n%s\n\n', ...
    fullfile(resultsDir, 'trajectory_prediction.png'));
