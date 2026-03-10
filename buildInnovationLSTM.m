function net = buildInnovationLSTM(seqLen)
    %buildInnovationLSTM 构建LSTM网络用于自适应因子预测
    %   net = buildInnovationLSTM(seqLen) 创建一个LSTM网络
    %
    %   seqLen: 输入序列长度(默认15)
    %
    %   返回: LSTM网络对象(layerGraph)
    %
    %   注意: 此版本与trainLSTMNetwork_v2.m匹配,使用简化结构

    if nargin < 1
        seqLen = 15;
    end
    
    % 简化的LSTM网络结构
    % 与trainLSTMNetwork_v2.m完全一致
    layers = [
        % 输入层: 特征维度为1
        sequenceInputLayer(1, "Name", "input")
        
        % LSTM层: 128个隐藏单元,输出最后一个时间步
        lstmLayer(128, "OutputMode", "last", "Name", "lstm1")
        dropoutLayer(0.2)
        
        % 全连接层1
        fullyConnectedLayer(64, "Name", "fc1")
        reluLayer
        dropoutLayer(0.2)
        
        % 全连接层2
        fullyConnectedLayer(32, "Name", "fc2")
        reluLayer
        
        % 输出层
        fullyConnectedLayer(1, "Name", "output")
        
        % 回归层(用于训练)
        regressionLayer("Name", "regression")
    ];
    
    % 返回层图(不转换为dlnetwork)
    net = layerGraph(layers);
    
    % 显示网络结构信息
    fprintf('LSTM网络结构已创建:\n');
    fprintf('  输入: 1个特征 × %d个时间步\n', seqLen);
    fprintf('  LSTM: 128个隐藏单元\n');
    fprintf('  全连接: 64 → 32 → 1\n');
    fprintf('  输出: 1个自适应因子 (0-1)\n');
end