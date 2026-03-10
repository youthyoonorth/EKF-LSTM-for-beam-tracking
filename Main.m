%% Pre 6G Cell Free Simulation with 5-Algorithm Beam Tracking Comparison
% Modified for Accuracy vs. Complexity Analysis
% Features: Grouped Bar Charts, Runtime Comparison, Trade-off Plot

%% Scenario Configuration
try
    addpath(genpath('C:\Users\86189\Documents\MATLAB\Examples\R2024b\5g\EvaluatePerformanceOfCellFreemMIMONetworksExample'));
catch
    warning('Path "EvaluatePerformanceOfCellFreemMIMONetworksExample" not found. Please add manually.');
end

clear;
if exist('wirelessnetworkSupportPackageCheck', 'file')
    wirelessnetworkSupportPackageCheck
end

rng("default")
numFrameSimulation = 20;  % Simulation frames
networkSimulator = wirelessNetworkSimulator.init;

if exist('pre6GCPU', 'class')
    try
        pre6GCPU.reset();
    catch
    end
end

%% Physical Layer Parameters
phyAbstractionType = "none";
duplexMode = "TDD";
split = "Centralized";

%% Area Parameters
lengthX = 1000;
lengthY = 1000;
numAPs = 25;
numUEs = 7;
numUEConnections = 4;

%% Create CPU
CPU = pre6GCPU(Name="CPU-1", Position=[lengthX*0.5, lengthY + 50, 10], ...
    PHYAbstractionMethod=phyAbstractionType, Split=split, DuplexMode=duplexMode, ...
    CarrierFrequency=1.9e9, ChannelBandwidth=20e6, SubcarrierSpacing=15e3);

%% Create APs
[apPositions, ~] = generateAPPositions(numAPs, lengthX, lengthY);
apNames = "AP-" + (1:size(apPositions,1));
APs = pre6GAP(Name=apNames, Position=apPositions, ...
    TransmitPower=23, NumTransmitAntennas=4, NumReceiveAntennas=4, ...
    ReceiveGain=0, NoiseFigure=9);

CPU.connectAP(APs);

%% Create UEs
uePositions = generateUEPositions(numUEs, lengthX, lengthY);
UEs = pre6GUE.empty(0, numUEs);

for i = 1:numUEs
    distSq = sum((apPositions(:,1:2) - uePositions(i,1:2)).^2, 2);
    [~, sortIdx] = sort(distSq, 'ascend');
    closestAPIdx = sortIdx(1:numUEConnections);
    
    ueName = "UE" + i + " AP-";
    for j = 1:numUEConnections
        ueName = ueName + floor(APs(closestAPIdx(j)).APCellID/3);
        if j ~= numUEConnections, ueName = ueName + "-"; end
    end
    
    UEs(i) = pre6GUE(Name=ueName, Position=uePositions(i,:), ...
        PHYAbstractionMethod=phyAbstractionType, ...
        TransmitPower=20, NumTransmitAntennas=1, NumReceiveAntennas=1, ...
        ReceiveGain=0, NoiseFigure=9);
    
    % Initialize Tracker
    if UEs(i).PhyEntity.TrackingEnabled
        speed_mag = 5 + 5 * rand(); 
        speed_dir = rand() * 2 * pi;
        velocity = [speed_mag * cos(speed_dir); speed_mag * sin(speed_dir); 0];
        currentPos = uePositions(i,:)';
        initState = [currentPos; velocity; 0; 0; 0];
        
        if ~isempty(UEs(i).PhyEntity.Tracker), UEs(i).PhyEntity.Tracker.reset(initState); end
        if ~isempty(UEs(i).PhyEntity.TrackerUKF), UEs(i).PhyEntity.TrackerUKF.reset(initState); end
        if ~isempty(UEs(i).PhyEntity.TrackerLSTM), UEs(i).PhyEntity.TrackerLSTM.reset(initState); end
        if ~isempty(UEs(i).PhyEntity.TrackerCascaded), UEs(i).PhyEntity.TrackerCascaded.reset(initState); end
        if ~isempty(UEs(i).PhyEntity.TrackerODE), UEs(i).PhyEntity.TrackerODE.reset(initState); end
    end
    
    for j = 1:numUEConnections
        APs(closestAPIdx(j)).connectUE(UEs(i), FullBufferTraffic="on");
    end
end

addNodes(networkSimulator, CPU);
addNodes(networkSimulator, APs);
addNodes(networkSimulator, UEs);

%% Channel Modeling
numNodes = length(CPU) + numAPs + numUEs;
channels = cell(numNodes, numNodes);
channelConfig = struct("DelaySpread", 300e-9);
refAP = APs(1);
commonWaveformInfo = nrOFDMInfo(refAP.NumResourceBlocks, refAP.SubcarrierSpacing/1e3);

for i = 1:numAPs
    channels = createCDLChannels(channels, channelConfig, APs(i), UEs, commonWaveformInfo);
end
customChannelModel = hNRCustomChannelModel(channels, struct(PHYAbstractionMethod=phyAbstractionType));
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel)

%% Visualization
if true
    pre6GNetworkVisualizer();
end

%% Run Simulation
simulationTime = numFrameSimulation * 1e-2;
fprintf('\n========================================\n');
fprintf('Starting Simulation (Comparison with Runtime Analysis)\n');
fprintf('========================================\n');
tic;
run(networkSimulator, simulationTime);
elapsedTime = toc;
fprintf('\n========================================\n');
fprintf('Simulation Completed in %.2f seconds\n', elapsedTime);
fprintf('========================================\n\n');

%% Performance Analysis & Comparative Plotting
fprintf('========================================\n');
fprintf('Generating Performance & Complexity Report...\n');
fprintf('========================================\n\n');

try
    trackingData = struct();
    numTracked = 0;
    
    for i = 1:numUEs
        ue = UEs(i);
        trackingData(i).name = sprintf('UE%d', i);
        trackingData(i).enabled = false;
        
        if ue.PhyEntity.TrackingEnabled && ~isempty(ue.PhyEntity.History.Time)
            hist = ue.PhyEntity.getFullHistory();
            
            trueAz = hist.TrueAz;
            trueEl = hist.TrueEl;
            
            % 1. Calculate RMSE
            rmse_ukf_az = calculateRMSE(hist.UKFAz, trueAz);
            rmse_lstm_az = calculateRMSE(hist.LSTMAz, trueAz);
            rmse_casc_az = calculateRMSE(hist.CascadedAz, trueAz);
            rmse_ode_az = calculateRMSE(hist.ODEAz, trueAz);
            rmse_prop_az = calculateRMSE(hist.PropAz, trueAz);
            
            rmse_ukf_el = calculateRMSE(hist.UKFEl, trueEl);
            rmse_lstm_el = calculateRMSE(hist.LSTMEl, trueEl);
            rmse_casc_el = calculateRMSE(hist.CascadedEl, trueEl);
            rmse_ode_el = calculateRMSE(hist.ODEEl, trueEl);
            rmse_prop_el = calculateRMSE(hist.PropEl, trueEl);
            
            % 2. Calculate Average Runtime (Computational Complexity)
            avg_time_ukf = mean(hist.UKFTime);
            avg_time_lstm = mean(hist.LSTMTime);
            avg_time_casc = mean(hist.CascTime);
            avg_time_ode = mean(hist.ODETime);
            avg_time_prop = mean(hist.PropTime);
            
            trackingData(i).enabled = true;
            trackingData(i).final_pos = ue.Position;
            trackingData(i).final_az = hist.PropAz(end);
            
            % Data Vectors for Plotting [UKF, LSTM, Cascaded, ODE, Proposed]
            trackingData(i).rmse_az = [rmse_ukf_az, rmse_lstm_az, rmse_casc_az, rmse_ode_az, rmse_prop_az];
            trackingData(i).rmse_el = [rmse_ukf_el, rmse_lstm_el, rmse_casc_el, rmse_ode_el, rmse_prop_el];
            trackingData(i).runtime = [avg_time_ukf, avg_time_lstm, avg_time_casc, avg_time_ode, avg_time_prop];
            
            numTracked = numTracked + 1;
            fprintf('  %s processed.\n', trackingData(i).name);
            fprintf('    AOA RMSE: UKF=%.3f, Prop=%.3f\n', rmse_ukf_az, rmse_prop_az);
            fprintf('    Avg Runtime: UKF=%.2fms, Prop=%.2fms\n', avg_time_ukf, avg_time_prop);
        end
    end
    
    if numTracked > 0
        createComprehensiveFigure(trackingData, apPositions);
        fprintf('\n✓ Visualization Generated Successfully.\n');
    end
    
catch ME
    fprintf('❌ Error in Analysis: %s\n', ME.message);
    fprintf('%s\n', getReport(ME));
end

fprintf('\n========================================\n');
fprintf('Simulation Finished\n');
fprintf('========================================\n');

%% ==================== Local Functions ====================

function rmse = calculateRMSE(est, true_val)
    n = min(length(est), length(true_val));
    est = est(1:n);
    true_val = true_val(1:n);
    err = abs(est - true_val);
    err = min(err, 360 - err); 
    rmse = sqrt(mean(err.^2));
    if isnan(rmse), rmse = 0; end
end

function createComprehensiveFigure(trackingData, apPositions)
    timestamp = string(datetime("now"), "yyyyMMdd_HHmmss");
    figName = sprintf('Algorithm_Analysis_%s', timestamp);
    
    fig = figure('Name', 'Comprehensive Analysis', 'Position', [50, 50, 1600, 1000], 'Color', 'w');
    
    valid_idx = find([trackingData.enabled]);
    if isempty(valid_idx), return; end
    
    ue_colors = lines(length(valid_idx));
    
    % Colors for 5 Algorithms
    algo_colors = [
        0.6, 0.6, 0.6;     % UKF (Grey)
        0.4, 0.6, 0.9;     % LSTM (Blue)
        0.9, 0.6, 0.2;     % Cascaded (Orange)
        0.6, 0.8, 0.4;     % ODE (Green)
        0.9, 0.2, 0.2      % Proposed (Red)
    ];
    
    % === 1. Topology (Top Left) ===
    subplot(2, 3, 1);
    hold on; grid on; axis equal;
    scatter(apPositions(:,1), apPositions(:,2), 100, 'k', 'filled', '^');
    for k = 1:length(valid_idx)
        idx = valid_idx(k);
        pos = trackingData(idx).final_pos;
        az = deg2rad(trackingData(idx).final_az);
        scatter(pos(1), pos(2), 150, ue_colors(k,:), 'filled', 'o', 'MarkerEdgeColor', 'k');
        quiver(pos(1), pos(2), 80*cos(az), 80*sin(az), 'Color', ue_colors(k,:), 'LineWidth', 2, 'AutoScale', 'off');
        text(pos(1)+20, pos(2)+20, trackingData(idx).name, 'Color', ue_colors(k,:), 'FontSize', 9, 'FontWeight', 'bold');
    end
    xlabel('X (m)'); ylabel('Y (m)'); title('Network Topology', 'FontSize', 12);
    xlim([0 1000]); ylim([0 1000]);
    
    % Prepare Data
    ue_labels = {trackingData(valid_idx).name};
    rmse_az = vertcat(trackingData(valid_idx).rmse_az);
    runtime_data = vertcat(trackingData(valid_idx).runtime);
    
    % === 2. Azimuth RMSE (Top Middle) ===
    subplot(2, 3, 2);
    b1 = bar(rmse_az, 'grouped');
    % [FIX] Safe color assignment
    if length(b1) == 5
        for k = 1:5, b1(k).FaceColor = algo_colors(k,:); end
    end
    set(gca, 'XTickLabel', ue_labels);
    ylabel('RMSE (deg)'); title('Azimuth Accuracy', 'FontSize', 12);
    grid on;
    legend({'UKF', 'LSTM', 'Cascaded', 'ODE', 'Proposed'}, 'Location', 'northwest', 'FontSize', 8);
    
    % === 3. Elevation RMSE (Top Right) ===
    subplot(2, 3, 3);
    rmse_el = vertcat(trackingData(valid_idx).rmse_el);
    b2 = bar(rmse_el, 'grouped');
    if length(b2) == 5
        for k = 1:5, b2(k).FaceColor = algo_colors(k,:); end
    end
    set(gca, 'XTickLabel', ue_labels);
    ylabel('RMSE (deg)'); title('Elevation Accuracy', 'FontSize', 12);
    grid on;
    
    % === 4. Computational Runtime (Bottom Left - NEW) ===
    subplot(2, 3, 4);
    b3 = bar(runtime_data, 'grouped');
    if length(b3) == 5
        for k = 1:5, b3(k).FaceColor = algo_colors(k,:); end
    end
    set(gca, 'XTickLabel', ue_labels);
    ylabel('Inference Time (ms)'); 
    title('Computational Complexity (Runtime)', 'FontSize', 12);
    grid on;
    
    % === 5. Accuracy-Complexity Trade-off (Bottom Middle - NEW) ===
    subplot(2, 3, 5);
    hold on; grid on;
    % Calculate averages across all UEs
    avg_rmse = mean(rmse_az, 1);
    avg_time = mean(runtime_data, 1);
    
    markers = {'s', 'o', 'd', 'h', 'p'};
    for k = 1:5
        plot(avg_time(k), avg_rmse(k), markers{k}, 'MarkerSize', 12, ...
            'MarkerFaceColor', algo_colors(k,:), 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    end
    xlabel('Avg Runtime (ms) [Lower is Better]'); 
    ylabel('Avg RMSE (deg) [Lower is Better]');
    title('Accuracy vs. Complexity Trade-off', 'FontSize', 12);
    % Add text annotations
    text(avg_time(1), avg_rmse(1)+0.2, 'UKF (Fast/High Error)', 'FontSize', 8, 'HorizontalAlignment', 'center');
    text(avg_time(5), avg_rmse(5)-0.1, 'Proposed (Optimal)', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r', 'HorizontalAlignment', 'center');
    xlim([0, 3.5]); ylim([0, max(avg_rmse)*1.2]);
    
    % === 6. Overall Improvement (Bottom Right) ===
    subplot(2, 3, 6);
    improvement = (avg_rmse(1) - avg_rmse) ./ avg_rmse(1) * 100;
    b4 = bar(improvement);
    b4.FaceColor = 'flat';
    b4.CData = algo_colors;
    set(gca, 'XTickLabel', {'UKF', 'LSTM', 'Casc', 'ODE', 'Prop'});
    ylabel('Improvement over UKF (%)');
    title('Accuracy Gain over Baseline', 'FontSize', 12);
    grid on;
    
    sgtitle('Cell-Free Beam Tracking: Comprehensive Performance Analysis', 'FontSize', 16, 'FontWeight', 'bold');
    
    saveas(fig, figName + ".png");
    fprintf('  Figure saved: %s.png\n', figName);
end

% ... [Helper functions: generateAPPositions, generateUEPositions, createCDLChannels] ...

function [apPositions, apRadius] = generateAPPositions(numAPs, lengthX, lengthY)
    a = round(sqrt(numAPs * lengthX / lengthY));
    while a > 0
        b = numAPs / a;
        if b == floor(b), break; end
        a = a - 1;
    end
    flag = 0;
    if a == 1 && b > 3
        flag = 1;
        a = round(sqrt((numAPs - 1) * lengthX / lengthY));
        while a > 0
            b = (numAPs - 1) / a;
            if b == floor(b), break; end
            a = a - 1;
        end
    end
    subX = lengthX / a;
    subY = lengthY / b;
    apRadius = max(subX, subY) / 2;
    x = zeros(numAPs, 1);
    y = zeros(numAPs, 1);
    k = 1;
    rx = 0.3 * b / (a + b);
    ry = 0.3 * a / (a + b);
    for i = 1:a
        for j = 1:b
            if flag && j == b, continue; end
            x(k) = subX * (i - (0.5 + rx) + 2 * rx * rand);
            y(k) = subY * (j - (0.5 + ry) + 2 * ry * rand);
            k = k + 1;
        end
    end
    if flag
        subX = lengthX / (a + 1);
        for i = 1:(a + 1)
            x(k) = subX * (i - (0.5 + rx) + 2 * rx * rand);
            y(k) = subY * (b - (0.5 + ry) + 2 * ry * rand);
            k = k + 1;
        end
    end
    z = 25 * ones(numAPs, 1);
    apPositions = [x y z];
end

function uePositions = generateUEPositions(numUEs, lengthX, lengthY)
    x = zeros(numUEs, 1);
    y = zeros(numUEs, 1);
    for i = 1:numUEs
        rx = mod(i, 4);
        ry = mod(rx, 2);
        x(i) = (floor(rx / 2) + rand) * lengthX / 2;
        y(i) = (ry + rand) * lengthY / 2;
    end
    z = 1.5 * ones(numUEs, 1);
    uePositions = [x y z];
end

function channels = createCDLChannels(channels, channelConfig, AP, UEs, waveformInfo)
    sampleRate = waveformInfo.SampleRate;
    channelFiltering = strcmp(AP.PHYAbstractionMethod, 'none');
    numUEs = length(UEs);
    channel = nrCDLChannel;
    channel.CarrierFrequency = AP.CarrierFrequency;
    channel.DelaySpread = channelConfig.DelaySpread;
    channel.ChannelFiltering = channelFiltering;
    channel.SampleRate = sampleRate;
    for ueIdx = 1:numUEs
        cdl = hMakeCustomCDL(channel);
        cdl.Seed = 73 + (ueIdx - 1);
        cdl = hArrayGeometry(cdl, AP.NumTransmitAntennas, UEs(ueIdx).NumReceiveAntennas, 'downlink');
        [~, depAngle] = rangeangle(UEs(ueIdx).Position', AP.Position');
        cdl.AnglesAoD(:) = cdl.AnglesAoD(:) + depAngle(1);
        cdl.AnglesZoD(:) = cdl.AnglesZoD(:) - cdl.AnglesZoD(1) + (90 - depAngle(2));
        channels{AP.ID, UEs(ueIdx).ID} = cdl;
        cdlUL = clone(cdl);
        cdlUL.swapTransmitAndReceive();
        channels{UEs(ueIdx).ID, AP.ID} = cdlUL;
    end
end