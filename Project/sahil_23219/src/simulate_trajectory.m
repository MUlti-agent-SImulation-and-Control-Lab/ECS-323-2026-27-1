function [t, xTrue, zTrue, xMeasured, zMeasured] = ...
    simulate_trajectory(params)
%SIMULATE_TRAJECTORY Generate a projectile trajectory and noisy measurements.
%
% Inputs:
%   params - structure containing trajectory and measurement parameters.
%
% Outputs: parameters and the variable assignement
%   t          - time vector
%   xTrue      - true horizontal position
%   zTrue      - true vertical position
%   xMeasured  - noisy horizontal measurements
%   zMeasured  - noisy vertical measurements

rng(params.randomSeed);

% Calculation of flight time.
a = -0.5 * params.g;
b = params.vz0;
c = params.z0;

rootsFlight = roots([a b c]);
positiveRoots = rootsFlight(rootsFlight > 0);
tLandTrue = max(positiveRoots);

% Sample the flight before landing.
t = linspace(0, tLandTrue, params.numSamples);

% True projectile path.
xTrue = params.x0 + params.vx0 .* t;
zTrue = params.z0 + params.vz0 .* t - ...
        0.5 .* params.g .* t.^2;

% Simulated sensor trials measurements.
noiseX = params.measurementNoiseStd .* randn(size(t));
noiseZ = params.measurementNoiseStd .* randn(size(t));

xMeasured = xTrue + noiseX;
zMeasured = zTrue + noiseZ;

end
