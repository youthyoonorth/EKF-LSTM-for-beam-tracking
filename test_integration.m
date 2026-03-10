%% 测试EKF-LSTM集成脚本
% 此脚本用于测试EKF-LSTM算法是否正确集成到CFmMIMO-SimDes平台

clear;
clc;

fprintf('========================================\n');
fprintf('EKF-LSTM集成测试\n');
fprintf('========================================\n');

%% 步骤1: 检查必需文件
fprintf('\n步骤 1: 检查必需文件...\n');

required_files = {
    'AdaptiveEKF.m', ...
    'EKFLSTMTracker.m', ...
    'buildInnovationLSTM.m', ...
    'trainLSTMNetwork_v2.m', ...
    'pre6GUEFullPHY.m'
};

all_files_exist = true;
for i = 1:length(required_files)
    if isfile(required_files{i})
        fprintf('  ✓ %s\n', required_files{i});
    else
        fprintf('  ✗ %s (缺失)\n', required_files{i});
        all_files_exist = false;
    end
end

if ~all_files_exist
    error('缺少必需文件,请检查!');
end

%% 步骤2: 检查LSTM网络
fprintf('\n步骤 2: 检查LSTM网络...\n');

if isfile('trained_lstm_net.mat')
    fprintf('  ✓ trained_lstm_net.mat 已存在\n');
    load('trained_lstm_net.mat', 'net');
    fprintf('  网络加载成功\n');
else
    fprintf('  ✗ trained_lstm_net.mat 不存在\n');
    fprintf('  正在训练LSTM网络...\n');
    trainLSTMNetwork();
    fprintf('  训练完成\n');
end

%% 步骤3: 测试EKF-LSTM跟踪器
fprintf('\n步骤 3: 测试EKF-LSTM跟踪器...\n');

try
    % 加载网络
    load('trained_lstm_net.mat', 'net');
    
    % 创建跟踪器
    dt = 0.001;  % 1ms
    base_q = 0.1;
    tracker = EKFLSTMTracker(dt, base_q, net);
    fprintf('  ✓ 跟踪器创建成功\n');
    
    % 初始化状态
    init_state = [100; 200; 0; 1; 0.5; 0; 0; 0; 0];  % [x,y,z,vx,vy,vz,ax,ay,az]
    tracker.reset(init_state);
    fprintf('  ✓ 跟踪器初始化成功\n');
    
    % 模拟跟踪过程
    fprintf('\n  模拟跟踪过程...\n');
    num_steps = 100;
    azimuth_history = zeros(num_steps, 1);
    elevation_history = zeros(num_steps, 1);
    
    for i = 1:num_steps
        % 模拟测量值
        t = (i-1) * dt;
        x = 100 + 1*t + 0.1*randn;
        y = 200 + 0.5*t + 0.1*randn;
        z = 0 + 0.1*randn;
        vx = 1 + 0.05*randn;
        vy = 0.5 + 0.05*randn;
        vz = 0.05*randn;
        
        measurement = [x; y; z; vx; vy; vz];
        
        % 执行跟踪
        [az, el] = tracker.step(measurement);
        azimuth_history(i) = az;
        elevation_history(i) = el;
    end
    
    fprintf('  ✓ 跟踪过程执行成功\n');
    
    % 显示结果
    fprintf('\n  跟踪结果统计:\n');
    fprintf('    方位角范围: [%.2f°, %.2f°]\n', min(azimuth_history), max(azimuth_history));
    fprintf('    俯仰角范围: [%.2f°, %.2f°]\n', min(elevation_history), max(elevation_history));
    
    % 绘制结果
    figure('Name', '跟踪器测试结果');
    
    subplot(2,1,1);
    plot(azimuth_history, 'b-', 'LineWidth', 1.5);
    title('方位角估计');
    xlabel('时间步'); ylabel('方位角 (度)');
    grid on;
    
    subplot(2,1,2);
    plot(elevation_history, 'r-', 'LineWidth', 1.5);
    title('俯仰角估计');
    xlabel('时间步'); ylabel('俯仰角 (度)');
    grid on;
    
    fprintf('\n  ✓ 跟踪器功能测试通过!\n');
    
catch ME
    fprintf('  ✗ 跟踪器测试失败: %s\n', ME.message);
    disp(ME.stack);
end

%% 步骤4: 测试AdaptiveEKF
fprintf('\n步骤 4: 测试AdaptiveEKF...\n');

try
    % 创建EKF
    dt = 0.001;
    base_q = 0.1;
    ekf = AdaptiveEKF(dt, base_q);
    fprintf('  ✓ AdaptiveEKF创建成功\n');
    
    % 测试预测和更新
    ekf.predict();
    fprintf('  ✓ 预测步骤执行成功\n');
    
    z = [100; 200; 0; 1; 0.5; 0];
    [x_post, innov] = ekf.update(z, 1.0);
    fprintf('  ✓ 更新步骤执行成功\n');
    fprintf('    Innovation: %.4f\n', innov);
    
    % 测试innovation序列
    seq = ekf.getInnovationSeq();
    fprintf('  ✓ Innovation序列获取成功 (长度: %d)\n', length(seq));
    
    fprintf('\n  ✓ AdaptiveEKF功能测试通过!\n');
    
catch ME
    fprintf('  ✗ AdaptiveEKF测试失败: %s\n', ME.message);
    disp(ME.stack);
end

%% 步骤5: 检查通信工具箱
fprintf('\n步骤 5: 检查MATLAB工具箱...\n');

% 检查通信工具箱
if license('test', 'Communication_Toolbox')
    fprintf('  ✓ 通信工具箱已安装\n');
else
    fprintf('  ✗ 通信工具箱未安装\n');
end

% 检查深度学习工具箱
if license('test', 'Neural_Network_Toolbox')
    fprintf('  ✓ 深度学习工具箱已安装\n');
else
    fprintf('  ✗ 深度学习工具箱未安装\n');
end

% 检查5G工具箱
if license('test', '5G_Toolbox')
    fprintf('  ✓ 5G工具箱已安装\n');
else
    fprintf('  ✗ 5G工具箱未安装\n');
end

%% 步骤6: 给出集成建议
fprintf('\n========================================\n');
fprintf('测试完成!\n');
fprintf('========================================\n');

fprintf('\n接下来的步骤:\n');
fprintf('1. 将 pre6GUEFullPHY_modified.m 重命名为 pre6GUEFullPHY.m 并替换原文件\n');
fprintf('2. 确保所有EKF-LSTM相关文件在MATLAB路径中\n');
fprintf('3. 运行Main.m启动仿真\n');
fprintf('4. 观察UE的跟踪性能\n');

fprintf('\n配置建议:\n');
fprintf('- 对于静止UE: TrackingEnabled = false\n');
fprintf('- 对于低速UE: TrackingInterval = 0.005 (5ms)\n');
fprintf('- 对于高速UE: TrackingInterval = 0.001 (1ms)\n');

fprintf('\n调试提示:\n');
fprintf('- 跟踪信息会每100ms打印一次\n');
fprintf('- 检查方位角和俯仰角的合理性\n');
fprintf('- 监控innovation值的变化\n');

fprintf('\n如有问题,请检查:\n');
fprintf('1. MATLAB版本 (建议R2023b或更高)\n');
fprintf('2. 所有必需工具箱是否安装\n');
fprintf('3. trained_lstm_net.mat是否存在\n');
fprintf('4. 文件路径配置是否正确\n');

fprintf('\n========================================\n');
