classdef LSTMOnlyTracker < handle
    %LSTMOnlyTracker Pure LSTM-based tracker for beam tracking
    %   纯LSTM跟踪器(无EKF),用于对比实验
    
    properties
        net                 % LSTM网络
        SEQ_LENGTH = 15     % 序列长度
        position_history = []  % 位置历史
        velocity_history = []  % 速度历史
        innovation_history = []
        current_position = [0; 0; 0]
        current_velocity = [0; 0; 0]
    end
    
    methods
        function obj = LSTMOnlyTracker(net)
            %LSTMOnlyTracker 构造函数
            obj.net = net;
        end
        
        function reset(obj, init_state)
            %reset 重置跟踪器状态
            obj.position_history = [];
            obj.velocity_history = [];
            obj.innovation_history = [];
            obj.current_position = init_state(1:3);
            obj.current_velocity = zeros(3, 1);
        end
        
        function [az, el] = step(obj, z)
            %step 执行一步LSTM预测
            
            % 提取测量位置和速度
            measured_pos = z(1:3);
            measured_vel = z(4:6);
            
            % 计算Innovation (测量与预测的差异)
            if isempty(obj.position_history)
                innovation = norm(measured_pos - obj.current_position);
            else
                predicted_pos = obj.current_position + obj.current_velocity * 0.001;
                innovation = norm(measured_pos - predicted_pos);
            end
            
            obj.innovation_history(end+1) = innovation;
            if length(obj.innovation_history) > obj.SEQ_LENGTH
                obj.innovation_history(1) = [];
            end
            
            % 使用LSTM预测自适应权重
            if length(obj.innovation_history) == obj.SEQ_LENGTH
                seq = obj.innovation_history(:);
                
                try
                    % 检测网络类型
                    if isa(obj.net, 'SeriesNetwork') || isa(obj.net, 'DAGNetwork')
                        X_input = reshape(seq, [1, obj.SEQ_LENGTH, 1]);
                        factor_raw = predict(obj.net, X_input);
                        adaptive_weight = double(factor_raw(1));
                    else
                        dlX = dlarray(seq', 'CTB');
                        factor_dl = predict(obj.net, dlX);
                        adaptive_weight = extractdata(factor_dl);
                        adaptive_weight = double(adaptive_weight(1));
                    end
                    
                    adaptive_weight = max(0, min(1, adaptive_weight));
                catch
                    adaptive_weight = 0.5;
                end
            else
                adaptive_weight = 0.5;
            end
            
            % 基于自适应权重融合测量值
            alpha = 0.3 + 0.6 * adaptive_weight;  % 0.3-0.9
            obj.current_position = alpha * measured_pos + (1-alpha) * obj.current_position;
            obj.current_velocity = alpha * measured_vel + (1-alpha) * obj.current_velocity;
            
            % 记录历史
            obj.position_history(:, end+1) = obj.current_position;
            obj.velocity_history(:, end+1) = obj.current_velocity;
            
            if size(obj.position_history, 2) > obj.SEQ_LENGTH
                obj.position_history(:, 1) = [];
                obj.velocity_history(:, 1) = [];
            end
            
            % 计算角度
            az = atan2d(obj.current_position(2), obj.current_position(1));
            el = atan2d(obj.current_position(3), ...
                hypot(obj.current_position(1), obj.current_position(2)));
        end
    end
end
