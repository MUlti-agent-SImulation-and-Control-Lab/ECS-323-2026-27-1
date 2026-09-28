function [landingX, landingT, zCoefficients] = ...
    predict_landing_point(t, xMeasured, zMeasured)
%PREDICT_LANDING_POINT Estimate the landing point from noisy measurements.
%
% A quadratic model is fitted to the measured vertical trajectory:
%
% z(t) = a*t^2 + b*t + c
%
% The positive f root of z(t)=0 is used as the predicted landing time.
% A linear model is fitted to x(t) to estimate horizontal position.

zCoefficients = polyfit(t, zMeasured, 2);

% Find intersections with ground z = 0.
groundRoots = roots(zCoefficients);

% Keep real positive roots.
validRoots = real(groundRoots(abs(imag(groundRoots)) < 1e-9 & ...
                              real(groundRoots) > 0));
% for muine errror message 
if isempty(validRoots)
    error('there is a error in the roots findin please check sahil.');
end

% Select the latest positive root as the landing time.
landingT = max(validRoots);

% Fit horizontal trajectory.
xCoefficients = polyfit(t, xMeasured, 1);

% Predict horizontal position at landing.
landingX = polyval(xCoefficients, landingT);

end
