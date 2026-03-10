%% Algorithm Comparison: UKF vs LSTM vs EKF-LSTM for Cell-Free Beam Tracking
% This script compares three beam tracking algorithms:
% 1. UKF (Unscented Kalman Filter)
% 2. Pure LSTM
% 3. EKF-LSTM (Proposed method)

clear; clc; close all;

fprintf('========================================\n');
fprintf('Beam Tracking Algorithm Comparison\n');
fprintf('========================================\n');
fprintf('Algorithms:\n');
fprintf('  1. UKF (Baseline)\n');
fprintf('  2. LSTM-Only\n');
fprintf('  3. EKF-LSTM (Proposed)\n');
fprintf('========================================\n\n');

%% Load LSTM Network
fprintf('Step 1: Loading LSTM network...\n');
if isfile('trained_lstm_net.mat')
    load('trained_lstm_net.mat', 'net');
    fprintf('  ✓ LSTM network loaded\n\n');
else
    error('trained_lstm_net.mat not found. Please train the network first.');
end

%% Simulation Parameters
numSteps = 200;      % Number of time steps
dt = 0.001;          % Time step (1ms)
base_q = 0.1;        % Base process noise

% UE initial positions (7 UEs)
ue_positions = [
    138.01, 839.85, 0;
    827.55, 81.31, 0;
    559.50, 749.18, 0;
    479.87, 170.19, 0;
    292.63, 611.91, 0;
    875.63, 127.55, 0;
    752.98, 849.54, 0
];

% AP positions (simplified - 4 APs for demo)
ap_positions = [
    200, 200, 10;
    800, 200, 10;
    200, 800, 10;
    800, 800, 10
];

numUEs = size(ue_positions, 1);

fprintf('Step 2: Simulation setup\n');
fprintf('  Number of UEs: %d\n', numUEs);
fprintf('  Number of APs: %d\n', size(ap_positions, 1));
fprintf('  Time steps: %d\n', numSteps);
fprintf('  Duration: %.2f seconds\n\n', numSteps * dt);

%% Initialize Trackers for All UEs
fprintf('Step 3: Initializing trackers...\n');

% Storage for all three algorithms
ukf_trackers = cell(numUEs, 1);
lstm_trackers = cell(numUEs, 1);
ekflstm_trackers = cell(numUEs, 1);

% Results storage
results = struct();
results.time = (0:numSteps-1) * dt;
results.ukf = struct();
results.lstm = struct();
results.ekflstm = struct();

for algo = {'ukf', 'lstm', 'ekflstm'}
    results.(algo{1}).azimuth = zeros(numUEs, numSteps);
    results.(algo{1}).elevation = zeros(numUEs, numSteps);
    results.(algo{1}).innovation = zeros(numUEs, numSteps);
end

% Initialize trackers
for ue_idx = 1:numUEs
    pos = ue_positions(ue_idx, :)';
    init_state = [pos; zeros(6,1)];
    
    % UKF
    ukf_trackers{ue_idx} = UKFTracker(dt, base_q);
    ukf_trackers{ue_idx}.reset(init_state);
    
    % Pure LSTM
    lstm_trackers{ue_idx} = LSTMOnlyTracker(net);
    lstm_trackers{ue_idx}.reset(init_state);
    
    % EKF-LSTM
    ekflstm_trackers{ue_idx} = EKFLSTMTracker(dt, base_q, net);
    ekflstm_trackers{ue_idx}.reset(init_state);
end

fprintf('  ✓ All trackers initialized\n\n');

%% Run Simulation
fprintf('Step 4: Running simulation...\n');

for step = 1:numSteps
    t = (step - 1) * dt;
    
    for ue_idx = 1:numUEs
        % Generate measurement (static UE + small noise)
        true_pos = ue_positions(ue_idx, :)';
        measured_pos = true_pos + randn(3,1) * 0.1;
        measured_vel = randn(3,1) * 0.05;
        measurement = [measured_pos; measured_vel];
        
        % Calculate true angle to nearest AP
        distances = sqrt(sum((ap_positions - true_pos').^2, 2));
        [~, nearest_ap] = min(distances);
        ap_pos = ap_positions(nearest_ap, :)';
        delta = ap_pos - true_pos;
        true_az = atan2d(delta(2), delta(1));
        true_el = atan2d(delta(3), hypot(delta(1), delta(2)));
        
        % UKF tracking
        [az_ukf, el_ukf] = ukf_trackers{ue_idx}.step(measurement);
        results.ukf.azimuth(ue_idx, step) = az_ukf;
        results.ukf.elevation(ue_idx, step) = el_ukf;
        if ~isempty(ukf_trackers{ue_idx}.innovation_history)
            results.ukf.innovation(ue_idx, step) = ...
                ukf_trackers{ue_idx}.innovation_history(end);
        end
        
        % Pure LSTM tracking
        [az_lstm, el_lstm] = lstm_trackers{ue_idx}.step(measurement);
        results.lstm.azimuth(ue_idx, step) = az_lstm;
        results.lstm.elevation(ue_idx, step) = el_lstm;
        if ~isempty(lstm_trackers{ue_idx}.innovation_history)
            results.lstm.innovation(ue_idx, step) = ...
                lstm_trackers{ue_idx}.innovation_history(end);
        end
        
        % EKF-LSTM tracking
        [az_ekflstm, el_ekflstm] = ekflstm_trackers{ue_idx}.step(measurement);
        results.ekflstm.azimuth(ue_idx, step) = az_ekflstm;
        results.ekflstm.elevation(ue_idx, step) = el_ekflstm;
        if ~isempty(ekflstm_trackers{ue_idx}.ekf.innovation_history)
            results.ekflstm.innovation(ue_idx, step) = ...
                ekflstm_trackers{ue_idx}.ekf.innovation_history(end);
        end
        
        % Store true values
        results.true_azimuth(ue_idx, step) = true_az;
        results.true_elevation(ue_idx, step) = true_el;
    end
    
    if mod(step, 50) == 0
        fprintf('  Progress: %d/%d steps (%.1f%%)\n', step, numSteps, 100*step/numSteps);
    end
end

fprintf('  ✓ Simulation completed\n\n');

%% Calculate Performance Metrics
fprintf('Step 5: Calculating performance metrics...\n');

metrics = struct();

for algo = {'ukf', 'lstm', 'ekflstm'}
    algo_name = algo{1};
    
    % Calculate RMSE for each UE
    az_errors = results.(algo_name).azimuth - results.true_azimuth;
    el_errors = results.(algo_name).elevation - results.true_elevation;
    
    % Handle angle wrapping for azimuth
    az_errors = wrapTo180(az_errors);
    
    metrics.(algo_name).azimuth_rmse = sqrt(mean(az_errors.^2, 2));
    metrics.(algo_name).elevation_rmse = sqrt(mean(el_errors.^2, 2));
    metrics.(algo_name).avg_innovation = mean(results.(algo_name).innovation, 2);
    
    % Overall metrics
    metrics.(algo_name).overall_az_rmse = sqrt(mean(az_errors(:).^2));
    metrics.(algo_name).overall_el_rmse = sqrt(mean(el_errors(:).^2));
    metrics.(algo_name).overall_innovation = mean(results.(algo_name).innovation(:));
end

fprintf('  ✓ Metrics calculated\n\n');

%% Print Performance Comparison
fprintf('========================================\n');
fprintf('Performance Comparison Summary\n');
fprintf('========================================\n\n');

fprintf('Overall Performance:\n');
fprintf('-------------------\n');
fprintf('Algorithm    | AOA RMSE (°) | AOD RMSE (°) | Avg Innovation\n');
fprintf('-------------|--------------|--------------|---------------\n');
fprintf('UKF          | %12.4f | %12.4f | %14.4f\n', ...
    metrics.ukf.overall_az_rmse, metrics.ukf.overall_el_rmse, metrics.ukf.overall_innovation);
fprintf('LSTM-Only    | %12.4f | %12.4f | %14.4f\n', ...
    metrics.lstm.overall_az_rmse, metrics.lstm.overall_el_rmse, metrics.lstm.overall_innovation);
fprintf('EKF-LSTM     | %12.4f | %12.4f | %14.4f\n', ...
    metrics.ekflstm.overall_az_rmse, metrics.ekflstm.overall_el_rmse, metrics.ekflstm.overall_innovation);
fprintf('\n');

% Calculate improvement percentage
ukf_az = metrics.ukf.overall_az_rmse;
ekflstm_az = metrics.ekflstm.overall_az_rmse;
improvement_az = (ukf_az - ekflstm_az) / ukf_az * 100;

ukf_el = metrics.ukf.overall_el_rmse;
ekflstm_el = metrics.ekflstm.overall_el_rmse;
improvement_el = (ukf_el - ekflstm_el) / ukf_el * 100;

fprintf('EKF-LSTM Improvement over UKF:\n');
fprintf('  AOA RMSE: %.2f%% reduction\n', improvement_az);
fprintf('  AOD RMSE: %.2f%% reduction\n', improvement_el);
fprintf('\n');

%% Visualization
fprintf('Step 6: Creating comparison plots...\n');

createAlgorithmComparisonFigure(results, metrics, ue_positions, ap_positions);

fprintf('  ✓ Plots created\n\n');

fprintf('========================================\n');
fprintf('Comparison completed!\n');
fprintf('========================================\n');

%% Save Results
save('algorithm_comparison_results.mat', 'results', 'metrics');
fprintf('Results saved to: algorithm_comparison_results.mat\n');

%% Visualization Function
function createAlgorithmComparisonFigure(results, metrics, ue_positions, ap_positions)
    %createAlgorithmComparisonFigure Create comprehensive comparison plots
    
    fig = figure('Name', 'Beam Tracking Algorithm Comparison', ...
        'Position', [50, 50, 1600, 1000], 'Color', 'w');
    
    numUEs = size(ue_positions, 1);
    colors_ue = lines(numUEs);
    
    % Algorithm colors
    color_ukf = [0.85, 0.33, 0.10];     % Red-orange
    color_lstm = [0.93, 0.69, 0.13];    % Yellow
    color_ekflstm = [0.00, 0.45, 0.74]; % Blue
    
    %% Subplot 1: Network Topology
    subplot(3, 3, [1, 4]);
    hold on; grid on; axis equal;
    
    % Plot APs
    h_ap = scatter(ap_positions(:,1), ap_positions(:,2), 180, 'k', 'filled', '^', ...
        'LineWidth', 1.5, 'DisplayName', sprintf('Access Points (N=%d)', size(ap_positions,1)));
    
    % Plot UEs
    h_ues = gobjects(numUEs, 1);
    for i = 1:numUEs
        pos = ue_positions(i, :);
        h_ues(i) = scatter(pos(1), pos(2), 220, colors_ue(i,:), 'filled', 'o', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2, ...
            'DisplayName', sprintf('UE%d', i));
        
        % Calculate angle to nearest AP
        distances = sqrt(sum((ap_positions - pos).^2, 2));
        [~, nearest_ap] = min(distances);
        ap_pos = ap_positions(nearest_ap, :);
        delta = ap_pos - pos;
        az_rad = atan2(delta(2), delta(1));
        
        % Draw beam direction
        arrow_length = 90;
        quiver(pos(1), pos(2), arrow_length*cos(az_rad), arrow_length*sin(az_rad), ...
            'Color', colors_ue(i,:), 'LineWidth', 3, 'MaxHeadSize', 1.5, ...
            'AutoScale', 'off', 'HandleVisibility', 'off');
        
        % Label
        text(pos(1)+25, pos(2)+25, sprintf('UE%d', i), ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', colors_ue(i,:));
    end
    
    xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Cell-Free Network Topology', 'FontSize', 14, 'FontWeight', 'bold');
    xlim([0, 1000]); ylim([0, 1000]);
    
    % Enhanced legend
    legend([h_ap, h_ues(1:min(3,numUEs))'], 'Location', 'best', ...
        'FontSize', 10, 'Box', 'on');
    
    %% Subplot 2: AOA RMSE Comparison
    subplot(3, 3, 2);
    hold on; grid on;
    
    x_pos = 1:numUEs;
    bar_width = 0.25;
    
    b1 = bar(x_pos - bar_width, metrics.ukf.azimuth_rmse, bar_width, ...
        'FaceColor', color_ukf, 'EdgeColor', 'k', 'LineWidth', 0.8, ...
        'DisplayName', 'UKF');
    b2 = bar(x_pos, metrics.lstm.azimuth_rmse, bar_width, ...
        'FaceColor', color_lstm, 'EdgeColor', 'k', 'LineWidth', 0.8, ...
        'DisplayName', 'LSTM-Only');
    b3 = bar(x_pos + bar_width, metrics.ekflstm.azimuth_rmse, bar_width, ...
        'FaceColor', color_ekflstm, 'EdgeColor', 'k', 'LineWidth', 0.8, ...
        'DisplayName', 'EKF-LSTM');
    
    set(gca, 'XTick', 1:numUEs, 'XTickLabel', arrayfun(@(x) sprintf('UE%d',x), 1:numUEs, 'UniformOutput', false));
    ylabel('RMSE (°)', 'FontSize', 11, 'FontWeight', 'bold');
    title('AOA Estimation Error (RMSE)', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    
    %% Subplot 3: AOD RMSE Comparison
    subplot(3, 3, 3);
    hold on; grid on;
    
    bar(x_pos - bar_width, metrics.ukf.elevation_rmse, bar_width, ...
        'FaceColor', color_ukf, 'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_pos, metrics.lstm.elevation_rmse, bar_width, ...
        'FaceColor', color_lstm, 'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_pos + bar_width, metrics.ekflstm.elevation_rmse, bar_width, ...
        'FaceColor', color_ekflstm, 'EdgeColor', 'k', 'LineWidth', 0.8);
    
    set(gca, 'XTick', 1:numUEs, 'XTickLabel', arrayfun(@(x) sprintf('UE%d',x), 1:numUEs, 'UniformOutput', false));
    ylabel('RMSE (°)', 'FontSize', 11, 'FontWeight', 'bold');
    title('AOD Estimation Error (RMSE)', 'FontSize', 12, 'FontWeight', 'bold');
    legend('UKF', 'LSTM-Only', 'EKF-LSTM', 'Location', 'best', 'FontSize', 9);
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    
    %% Subplot 4: Overall RMSE Comparison
    subplot(3, 3, 5);
    hold on; grid on;
    
    categories = {'AOA RMSE', 'AOD RMSE'};
    ukf_overall = [metrics.ukf.overall_az_rmse, metrics.ukf.overall_el_rmse];
    lstm_overall = [metrics.lstm.overall_az_rmse, metrics.lstm.overall_el_rmse];
    ekflstm_overall = [metrics.ekflstm.overall_az_rmse, metrics.ekflstm.overall_el_rmse];
    
    x_cat = 1:length(categories);
    bar(x_cat - bar_width, ukf_overall, bar_width, 'FaceColor', color_ukf, ...
        'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_cat, lstm_overall, bar_width, 'FaceColor', color_lstm, ...
        'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_cat + bar_width, ekflstm_overall, bar_width, 'FaceColor', color_ekflstm, ...
        'EdgeColor', 'k', 'LineWidth', 0.8);
    
    set(gca, 'XTick', 1:length(categories), 'XTickLabel', categories);
    ylabel('RMSE (°)', 'FontSize', 11, 'FontWeight', 'bold');
    title('Overall Performance Comparison', 'FontSize', 12, 'FontWeight', 'bold');
    legend('UKF', 'LSTM-Only', 'EKF-LSTM', 'Location', 'best', 'FontSize', 9);
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    
    %% Subplot 5: Innovation Comparison
    subplot(3, 3, 6);
    hold on; grid on;
    
    bar(x_pos - bar_width, metrics.ukf.avg_innovation, bar_width, ...
        'FaceColor', color_ukf, 'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_pos, metrics.lstm.avg_innovation, bar_width, ...
        'FaceColor', color_lstm, 'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_pos + bar_width, metrics.ekflstm.avg_innovation, bar_width, ...
        'FaceColor', color_ekflstm, 'EdgeColor', 'k', 'LineWidth', 0.8);
    
    set(gca, 'XTick', 1:numUEs, 'XTickLabel', arrayfun(@(x) sprintf('UE%d',x), 1:numUEs, 'UniformOutput', false));
    ylabel('Innovation', 'FontSize', 11, 'FontWeight', 'bold');
    title('Average Innovation (Measurement Residual)', 'FontSize', 12, 'FontWeight', 'bold');
    legend('UKF', 'LSTM-Only', 'EKF-LSTM', 'Location', 'best', 'FontSize', 9);
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    
    %% Subplot 6: Time-series example (UE1)
    subplot(3, 3, 7);
    hold on; grid on;
    
    ue_idx = 1;
    plot(results.time, results.ukf.azimuth(ue_idx, :), '-', ...
        'Color', color_ukf, 'LineWidth', 2, 'DisplayName', 'UKF');
    plot(results.time, results.lstm.azimuth(ue_idx, :), '--', ...
        'Color', color_lstm, 'LineWidth', 2, 'DisplayName', 'LSTM-Only');
    plot(results.time, results.ekflstm.azimuth(ue_idx, :), '-', ...
        'Color', color_ekflstm, 'LineWidth', 2.5, 'DisplayName', 'EKF-LSTM');
    plot(results.time, results.true_azimuth(ue_idx, :), 'k:', ...
        'LineWidth', 1.5, 'DisplayName', 'True');
    
    xlabel('Time (s)', 'FontSize', 10);
    ylabel('Azimuth (°)', 'FontSize', 10);
    title(sprintf('AOA Tracking (UE%d)', ue_idx), 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 8);
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    
    %% Subplot 7: Improvement Percentage
    subplot(3, 3, 8);
    hold on; grid on;
    
    % Calculate improvements over UKF
    lstm_improvement = (metrics.ukf.azimuth_rmse - metrics.lstm.azimuth_rmse) ./ ...
        metrics.ukf.azimuth_rmse * 100;
    ekflstm_improvement = (metrics.ukf.azimuth_rmse - metrics.ekflstm.azimuth_rmse) ./ ...
        metrics.ukf.azimuth_rmse * 100;
    
    bar(x_pos - bar_width/2, lstm_improvement, bar_width, ...
        'FaceColor', color_lstm, 'EdgeColor', 'k', 'LineWidth', 0.8);
    bar(x_pos + bar_width/2, ekflstm_improvement, bar_width, ...
        'FaceColor', color_ekflstm, 'EdgeColor', 'k', 'LineWidth', 0.8);
    
    yline(0, 'k--', 'LineWidth', 1);
    set(gca, 'XTick', 1:numUEs, 'XTickLabel', arrayfun(@(x) sprintf('UE%d',x), 1:numUEs, 'UniformOutput', false));
    ylabel('Improvement (%)', 'FontSize', 11, 'FontWeight', 'bold');
    title('AOA RMSE Improvement over UKF', 'FontSize', 12, 'FontWeight', 'bold');
    legend('LSTM-Only', 'EKF-LSTM', 'Location', 'best', 'FontSize', 9);
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    
    %% Subplot 8: Performance Summary Table
    subplot(3, 3, 9);
    axis off;
    
    summaryText = sprintf('Performance Summary\n');
    summaryText = sprintf('%s━━━━━━━━━━━━━━━━━━━━━━━\n\n', summaryText);
    
    summaryText = sprintf('%sAlgorithm Comparison:\n\n', summaryText);
    
    summaryText = sprintf('%sUKF (Baseline):\n', summaryText);
    summaryText = sprintf('%s  AOA RMSE: %.4f°\n', summaryText, metrics.ukf.overall_az_rmse);
    summaryText = sprintf('%s  AOD RMSE: %.4f°\n\n', summaryText, metrics.ukf.overall_el_rmse);
    
    summaryText = sprintf('%sLSTM-Only:\n', summaryText);
    summaryText = sprintf('%s  AOA RMSE: %.4f°\n', summaryText, metrics.lstm.overall_az_rmse);
    summaryText = sprintf('%s  AOD RMSE: %.4f°\n', summaryText, metrics.lstm.overall_el_rmse);
    lstm_improv = (metrics.ukf.overall_az_rmse - metrics.lstm.overall_az_rmse) / ...
        metrics.ukf.overall_az_rmse * 100;
    summaryText = sprintf('%s  Improvement: %.2f%%\n\n', summaryText, lstm_improv);
    
    summaryText = sprintf('%sEKF-LSTM (Proposed):\n', summaryText);
    summaryText = sprintf('%s  AOA RMSE: %.4f°\n', summaryText, metrics.ekflstm.overall_az_rmse);
    summaryText = sprintf('%s  AOD RMSE: %.4f°\n', summaryText, metrics.ekflstm.overall_el_rmse);
    ekflstm_improv = (metrics.ukf.overall_az_rmse - metrics.ekflstm.overall_az_rmse) / ...
        metrics.ukf.overall_az_rmse * 100;
    summaryText = sprintf('%s  Improvement: %.2f%%\n\n', summaryText, ekflstm_improv);
    
    summaryText = sprintf('%s━━━━━━━━━━━━━━━━━━━━━━━\n', summaryText);
    summaryText = sprintf('%sKey Findings:\n', summaryText);
    if ekflstm_improv > lstm_improv
        summaryText = sprintf('%s✓ EKF-LSTM outperforms\n', summaryText);
        summaryText = sprintf('%s  both UKF and LSTM\n', summaryText);
    else
        summaryText = sprintf('%s- Performance varies\n', summaryText);
        summaryText = sprintf('%s  by UE location\n', summaryText);
    end
    
    text(0.05, 0.95, summaryText, ...
        'VerticalAlignment', 'top', ...
        'FontName', 'FixedWidth', ...
        'FontSize', 8.5, ...
        'Interpreter', 'none');
    
    % Main title
    sgtitle('Cell-Free Beam Tracking: Algorithm Performance Comparison', ...
        'FontSize', 16, 'FontWeight', 'bold');
    
    % Save figure
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    figname = sprintf('Algorithm_Comparison_%s.png', timestamp);
    saveas(fig, figname);
    fprintf('  Figure saved: %s\n', figname);
end
