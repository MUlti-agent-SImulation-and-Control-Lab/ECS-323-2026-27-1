%% Finger Angle vs Time

clear; clc; close all;

%% --- Load data ---
filename = 'pinn_data.csv';
opts = detectImportOptions(filename);
T = readtable(filename, opts);

t  = T.time_ms;
sv = T.servo_angle;
fa = T.finger_angle;
N  = height(T);

%% Smooth the finger-angle signal to get a clean trend ---
smoothWin = 101;                 % samples (~2 s) - removes sensor noise
fSmooth = movmedian(fa, smoothWin);

%% Estimate local slope (deg/ms) with a wide centered window ---
slopeWin = 150;                  % half-window, samples (~3 s each side)
idxAll = (1:N)';
loIdx = max(1, idxAll - slopeWin);
hiIdx = min(N, idxAll + slopeWin);
dtWin = t(hiIdx) - t(loIdx);
dtWin(dtWin == 0) = NaN;         % avoid divide-by-zero
slopeEst = (fSmooth(hiIdx) - fSmooth(loIdx)) ./ dtWin;
slopeEst(isnan(slopeEst)) = 0;

%% Classify each sample as rise / fall / flat ---
slopeThresh = 3e-4;              % deg/ms (~0.3 deg/s) - tuned to this data's ramp rate
stateNum = zeros(N,1);
stateNum(slopeEst >  slopeThresh) = 1;    % rise
stateNum(slopeEst < -slopeThresh) = -1;   % fall
% (0 = flat)

% Smooth the classification
stateSmooth = round(movmedian(stateNum, 201));

%% Group into contiguous segments, merge short ones
changeIdx = [1; find(diff(stateSmooth) ~= 0) + 1; N + 1];
segStarts = changeIdx(1:end-1);
segEnds   = changeIdx(2:end) - 1;
segLabel  = stateSmooth(segStarts);   % 1 = rise, -1 = fall, 0 = flat

minSegMs = 9000;  
k = 1;
while k <= numel(segStarts)
    dur = t(segEnds(k)) - t(segStarts(k));
    if dur < minSegMs && numel(segStarts) > 1
        if k < numel(segStarts)
            segStarts(k+1) = segStarts(k);
        else
            segEnds(k-1) = segEnds(k);
        end
        segStarts(k) = [];
        segEnds(k)   = [];
        segLabel(k)  = [];
    else
        k = k + 1;
    end
end
nSeg = numel(segStarts);

%% Fit each segment and plot
figure('Name', 'Automatic Triangle-Flat-Triangle Fit', 'Position', [100 100 1000 650]);
hold on;

riseColor = [0.20 0.55 0.95];
fallColor = [0.90 0.35 0.20];
flatColor = [0.95 0.75 0.10];

results = struct('type', {}, 'tRange', {}, 'servoRange', {}, 'slope', {}, 'intercept', {}, 'R2', {}, 'meanVal', {}, ...
    'tLine', {}, 'yLine', {}, 'color', {});

for k = 1:nSeg
    idx1 = segStarts(k);
    idx2 = segEnds(k);

    xk = sv(idx1:idx2);
    yk = fa(idx1:idx2);
    tk = t(idx1:idx2);
    servoRange = [min(xk), max(xk)];

    if segLabel(k) == 0
        % Flat / plateau segment: horizontal line at the mean level
        typeStr = 'flat';
        color = flatColor;
        yLevel = mean(yk);
        tLine = [tk(1), tk(end)];
        yLine = [yLevel, yLevel];

        results(end+1) = struct('type', typeStr, 'tRange', [tk(1) tk(end)], ...
            'servoRange', servoRange, 'slope', NaN, 'intercept', NaN, 'R2', NaN, ...
            'meanVal', yLevel, 'tLine', tLine, 'yLine', yLine, 'color', color); %#ok<AGROW>
    else
        % Rising or falling ramp: linear regression finger vs servo
        p = polyfit(xk, yk, 1);
        m = p(1); c = p(2);
        yHat = polyval(p, xk);
        SSres = sum((yk - yHat).^2);
        SStot = sum((yk - mean(yk)).^2);
        R2 = 1 - SSres / SStot;

        tLine = [tk(1), tk(end)];
        yLine = polyval(p, [xk(1), xk(end)]);

        if segLabel(k) == 1
            typeStr = 'rise'; color = riseColor;
        else
            typeStr = 'fall'; color = fallColor;
        end
        results(end+1) = struct('type', typeStr, 'tRange', [tk(1) tk(end)], ...
            'servoRange', servoRange, 'slope', m, 'intercept', c, 'R2', R2, ...
            'meanVal', NaN, 'tLine', tLine, 'yLine', yLine, 'color', color); %#ok<AGROW>
    end
end

%% Force visual continuity between adjacent segments
forceContinuous = true;
if forceContinuous
    for k = 1:nSeg-1
        boundaryVal = (results(k).yLine(2) + results(k+1).yLine(1)) / 2;
        results(k).yLine(2)   = boundaryVal;
        results(k+1).yLine(1) = boundaryVal;
    end
end

%% Plot
for k = 1:nSeg
    r = results(k);
    idx1 = segStarts(k);
    idx2 = segEnds(k);
    tk = t(idx1:idx2);
    yk = fa(idx1:idx2);

    switch r.type
        case 'flat'
            legendName = sprintf('Flat (servo %.0f-%.0f)', r.servoRange(1), r.servoRange(2));
        case 'rise'
            legendName = sprintf('Rising ramp (servo %.0f-%.0f)', r.servoRange(1), r.servoRange(2));
        case 'fall'
            legendName = sprintf('Falling ramp (servo %.0f-%.0f)', r.servoRange(1), r.servoRange(2));
    end

    scatter(tk, yk, 6, r.color, 'filled', 'MarkerFaceAlpha', 0.2, 'HandleVisibility', 'off');
    plot(r.tLine, r.yLine, 'Color', r.color, 'LineWidth', 2.2, 'DisplayName', legendName);
end

hold off;
grid on;
xlabel('Time (ms)');
ylabel('Finger Angle (deg)');
title('Finger Angle vs Time - Automatic Triangle / Flat / Triangle Fit');
legend('Location', 'best');

%% Print summary
fprintf('\n--- Segment Summary (chronological) ---\n');
for k = 1:numel(results)
    r = results(k);
    if strcmp(r.type, 'flat')
        fprintf('[FLAT ] t=%7d-%7d ms : finger ~= %.2f deg (constant)\n', ...
            r.tRange(1), r.tRange(2), r.meanVal);
    else
        fprintf('[%-4s] t=%7d-%7d ms : finger = %.4f*servo + %.4f   (R^2=%.4f)\n', ...
            upper(r.type), r.tRange(1), r.tRange(2), r.slope, r.intercept, r.R2);
    end
end
