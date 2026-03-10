classdef EKFLSTMTracker < handle
    properties
        ekf
        net
        SEQ_LENGTH = 15
        useRegressionLayer = false
    end

    methods
        function obj = EKFLSTMTracker(dt, base_q, net)
            obj.ekf = AdaptiveEKF(dt, base_q);
            obj.net = net;
            
            % 检测网络类型
            if isa(net, 'SeriesNetwork') || isa(net, 'DAGNetwork')
                obj.useRegressionLayer = true;
            elseif isa(net, 'dlnetwork')
                obj.useRegressionLayer = false;
            else
                warning('Unknown network type, assuming dlnetwork');
                obj.useRegressionLayer = false;
            end
        end

        function reset(obj, init_state)
            obj.ekf.x(1:length(init_state)) = init_state(:);
            obj.ekf.innovation_history = [];
        end

        function [az, el] = step(obj, z)
            % EKF 预测
            obj.ekf.predict();
        
            % 自适应因子
            if length(obj.ekf.innovation_history) == obj.SEQ_LENGTH
                seq = obj.ekf.getInnovationSeq();
                
                try
                    if obj.useRegressionLayer
                        X_input = reshape(seq, [1, obj.SEQ_LENGTH, 1]);
                        factor_raw = predict(obj.net, X_input);
                        factor = double(factor_raw(1));
                    else
                        dlX = dlarray(seq', 'CTB');
                        factor_dl = predict(obj.net, dlX);
                        factor = extractdata(factor_dl);
                        factor = double(factor(1));
                    end
                    
                    factor = max(0, min(1, factor));
                    adaptive_factor = 0.5 + 2.5 * factor;
                    
                catch ME
                    warning('LSTM prediction failed: %s. Using default factor.', ME.message);
                    adaptive_factor = 1.0;
                end
            else
                adaptive_factor = 1.0;
            end
        
            % EKF 更新
            obj.ekf.update(z, adaptive_factor);
        
            % 输出角度
            x = obj.ekf.x;
            az = atan2d(x(2), x(1));
            el = atan2d(x(3), hypot(x(1), x(2)));
        end
    end
end