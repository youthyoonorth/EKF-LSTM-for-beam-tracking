function trainLSTMNetwork_v2()
    %trainLSTMNetwork_v2 训练LSTM网络用于自适应因子预测 (修复版)
    %
    % 修复说明：
    % 将输入数据格式从 3D 数组改为 Cell Array，解决了维度识别错误的问题。
    
    fprintf('========================================\n');
    fprintf('开始训练LSTM网络 (修复版)\n');
    fprintf('========================================\n');
    
    %% 1. 生成训练数据
    fprintf('\n步骤 1: 生成训练数据...\n');
    
    numSamples = 5000;   % 样本数
    seqLength = 15;      % 序列长度
    
    % [修改 1] 使用元胞数组 (Cell Array) 存储序列数据
    % 格式：numSamples x 1 的元胞数组，每个元胞内是 1 x seqLength 的矩阵
    X_train = cell(numSamples, 1); 
    Y_train = zeros(numSamples, 1);
    
    fprintf('生成 %d 个训练样本...\n', numSamples);
    
    for i = 1:numSamples
        scenario_type = rand;
        
        if scenario_type < 0.3
            % 场景1: 低动态
            base_value = 0.5 + rand * 0.5;
            noise_level = 0.1;
            seq = base_value + randn(seqLength, 1) * noise_level;
            target = 0.2;
            
        elseif scenario_type < 0.7
            % 场景2: 中等动态
            base_value = 2.0 + rand * 2.0;
            noise_level = 0.5;
            seq = base_value + randn(seqLength, 1) * noise_level;
            trend = linspace(0, rand-0.5, seqLength)';
            seq = seq + trend;
            target = 0.5;
            
        else
            % 场景3: 高动态
            base_value = 5.0 + rand * 5.0;
            noise_level = 1.5;
            seq = base_value + randn(seqLength, 1) * noise_level;
            if rand > 0.5
                change_point = randi([5, 12]);
                seq(change_point:end) = seq(change_point:end) + (rand-0.5)*5;
            end
            target = 0.9;
        end
        
        % 确保序列值为正
        seq = abs(seq);
        
        % [修改 1] 存储到元胞数组中
        % 每个序列格式为: 1 (特征) x 15 (时间步)
        X_train{i} = seq'; 
        Y_train(i) = target;
        
        if mod(i, 500) == 0
            fprintf('  已生成 %d/%d 样本\n', i, numSamples);
        end
    end
    
    fprintf('训练数据生成完成!\n');
    
    %% 2. 划分训练集和验证集
    fprintf('\n步骤 2: 划分数据集...\n');
    
    numTrain = floor(0.8 * numSamples);
    numVal = numSamples - numTrain;
    
    idx_shuffle = randperm(numSamples);
    train_idx = idx_shuffle(1:numTrain);
    val_idx = idx_shuffle(numTrain+1:end);
    
    % [修改 2] 元胞数组的切片方式
    X_train_set = X_train(train_idx);
    Y_train_set = Y_train(train_idx);
    X_val_set = X_train(val_idx);
    Y_val_set = Y_train(val_idx);
    
    fprintf('训练集: %d 样本, 验证集: %d 样本\n', numTrain, numVal);
    
    %% 3. 构建简化的LSTM网络
    fprintf('\n步骤 3: 构建LSTM网络...\n');
    
    layers = [
        % 输入层：特征维度为 1
        sequenceInputLayer(1, "Name", "input")
        
        lstmLayer(128, "OutputMode", "last", "Name", "lstm1")
        dropoutLayer(0.2)
        
        fullyConnectedLayer(64, "Name", "fc1")
        reluLayer
        dropoutLayer(0.2)
        
        fullyConnectedLayer(32, "Name", "fc2")
        reluLayer
        
        fullyConnectedLayer(1, "Name", "output")
        regressionLayer("Name", "regression")
    ];
    
    fprintf('网络结构创建完成\n');
    
    %% 4. 设置训练选项
    fprintf('\n步骤 4: 配置训练参数...\n');
    
    options = trainingOptions('adam', ...
        'MaxEpochs', 30, ...
        'MiniBatchSize', 64, ...
        'InitialLearnRate', 0.001, ...
        'LearnRateSchedule', 'piecewise', ...
        'LearnRateDropFactor', 0.5, ...
        'LearnRateDropPeriod', 10, ...
        'Shuffle', 'every-epoch', ...
        'ValidationData', {X_val_set, Y_val_set}, ...
        'ValidationFrequency', 30, ...
        'Verbose', true, ...
        'VerboseFrequency', 30, ...
        'Plots', 'training-progress', ...
        'ExecutionEnvironment', 'auto');
    
    fprintf('训练参数配置完成\n');
    
    %% 5. 训练网络
    fprintf('\n步骤 5: 开始训练网络...\n');
    
    try
        % 训练网络
        net = trainNetwork(X_train_set, Y_train_set, layers, options);
        
        fprintf('========================================\n');
        fprintf('网络训练完成!\n');
        
        %% 6. 评估网络性能
        fprintf('\n步骤 6: 评估网络性能...\n');
        
        % 在验证集上评估
        Y_pred_val = predict(net, X_val_set);
        
        % 计算评估指标
        mse = mean((Y_pred_val - Y_val_set).^2);
        mae = mean(abs(Y_pred_val - Y_val_set));
        rmse = sqrt(mse);
        
        fprintf('验证集性能:\n');
        fprintf('  - MSE:  %.6f\n', mse);
        fprintf('  - MAE:  %.6f\n', mae);
        fprintf('  - RMSE: %.6f\n', rmse);
        
        % 绘制预测结果
        figure('Name', 'Prediction Results', 'Position', [100, 100, 1200, 800]);
        
        subplot(2,3,1);
        plot(Y_val_set, 'b-', 'LineWidth', 1.5, 'DisplayName', '真实值');
        hold on;
        plot(Y_pred_val, 'r--', 'LineWidth', 1.5, 'DisplayName', '预测值');
        legend('Location', 'best');
        title('验证集预测结果');
        xlabel('样本索引'); ylabel('自适应因子');
        grid on;
        
        subplot(2,3,2);
        scatter(Y_val_set, Y_pred_val, 20, 'filled');
        hold on;
        plot([0 1], [0 1], 'r--', 'LineWidth', 2);
        title('预测值 vs 真实值');
        xlabel('真实值'); ylabel('预测值');
        axis equal; axis([0 1 0 1]);
        grid on;
        
        subplot(2,3,3);
        errors = Y_pred_val - Y_val_set;
        histogram(errors, 30);
        title('预测误差分布');
        xlabel('误差'); ylabel('频数');
        grid on;
        
        subplot(2,3,4);
        plot(abs(errors), 'b-');
        title('绝对误差');
        xlabel('样本索引'); ylabel('|误差|');
        grid on;
        
        % [修改 3] 绘图部分适配 Cell Array
        subplot(2,3,5);
        idx_low = find(Y_val_set < 0.3, 1);
        if ~isempty(idx_low)
            % 使用 {} 索引访问元胞内容
            plot(X_val_set{idx_low}, 'b-', 'LineWidth', 1.5);
            title(sprintf('低动态样本 (真实=%.2f, 预测=%.2f)', Y_val_set(idx_low), Y_pred_val(idx_low)));
            xlabel('时间步'); ylabel('Innovation值');
            grid on;
        end
        
        subplot(2,3,6);
        idx_high = find(Y_val_set > 0.8, 1);
        if ~isempty(idx_high)
            % 使用 {} 索引访问元胞内容
            plot(X_val_set{idx_high}, 'r-', 'LineWidth', 1.5);
            title(sprintf('高动态样本 (真实=%.2f, 预测=%.2f)', Y_val_set(idx_high), Y_pred_val(idx_high)));
            xlabel('时间步'); ylabel('Innovation值');
            grid on;
        end
        
        %% 7. 保存网络
        fprintf('\n步骤 7: 保存训练好的网络...\n');
        
        save('trained_lstm_net.mat', 'net', 'mse', 'mae', 'rmse');
        fprintf('网络已保存到: trained_lstm_net.mat\n');
        fprintf('训练成功完成!\n');
        
    catch ME
        fprintf('\n训练过程出错: %s\n', ME.message);
        fprintf('错误堆栈:\n');
        for k = 1:length(ME.stack)
            fprintf('  文件: %s\n', ME.stack(k).file);
            fprintf('  函数: %s\n', ME.stack(k).name);
            fprintf('  行号: %d\n', ME.stack(k).line);
        end
    end
end