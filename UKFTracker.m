classdef UKFTracker < handle
    %UKFTracker Unscented Kalman Filter for beam tracking
    %   基于UKF的波束追踪器,用于对比实验
    
    properties
        dt              % 时间步长
        x               % 状态向量 [x,y,z,vx,vy,vz,ax,ay,az]
        P               % 协方差矩阵
        Q               % 过程噪声
        R               % 测量噪声
        alpha = 1e-3    % UKF参数
        beta = 2        % UKF参数
        kappa = 0       % UKF参数
        innovation_history = []
    end
    
    methods
        function obj = UKFTracker(dt, base_q)
            %UKFTracker 构造函数
            obj.dt = dt;
            
            % 初始化状态
            obj.x = zeros(9, 1);
            obj.P = eye(9) * 10;
            
            % 过程噪声
            obj.Q = eye(9) * base_q;
            obj.Q(7:9, 7:9) = 2 * obj.Q(7:9, 7:9);
            
            % 测量噪声
            obj.R = eye(6) * 5;
        end
        
        function reset(obj, init_state)
            %reset 重置跟踪器状态
            obj.x(1:length(init_state)) = init_state(:);
            obj.P = eye(9) * 10;
            obj.innovation_history = [];
        end
        
        function [az, el] = step(obj, z)
            %step 执行一步UKF预测和更新
            
            % UKF预测
            obj.predict();
            
            % UKF更新
            obj.update(z);
            
            % 计算角度
            az = atan2d(obj.x(2), obj.x(1));
            el = atan2d(obj.x(3), hypot(obj.x(1), obj.x(2)));
        end
        
        function predict(obj)
            %predict UKF预测步骤
            
            n = length(obj.x);
            
            % 计算Sigma点参数
            lambda = obj.alpha^2 * (n + obj.kappa) - n;
            
            % 生成Sigma点
            sqrtP = chol((n + lambda) * obj.P, 'lower');
            X = repmat(obj.x, 1, 2*n+1);
            X(:, 2:n+1) = X(:, 2:n+1) + sqrtP;
            X(:, n+2:end) = X(:, n+2:end) - sqrtP;
            
            % 状态转移函数
            F = eye(9);
            for i = 1:3
                F(i, i+3) = obj.dt;
                F(i, i+6) = 0.5 * obj.dt^2;
                F(i+3, i+6) = obj.dt;
            end
            
            % 传播Sigma点
            X_pred = zeros(size(X));
            for i = 1:size(X, 2)
                X_pred(:, i) = F * X(:, i);
            end
            
            % 计算权重
            Wm = ones(1, 2*n+1) / (2*(n+lambda));
            Wc = Wm;
            Wm(1) = lambda / (n + lambda);
            Wc(1) = lambda / (n + lambda) + (1 - obj.alpha^2 + obj.beta);
            
            % 预测均值
            obj.x = X_pred * Wm';
            
            % 预测协方差
            obj.P = obj.Q;
            for i = 1:size(X_pred, 2)
                diff = X_pred(:, i) - obj.x;
                obj.P = obj.P + Wc(i) * (diff * diff');
            end
        end
        
        function update(obj, z)
            %update UKF更新步骤
            
            z = z(:);
            n = length(obj.x);
            m = length(z);
            
            % 计算Sigma点参数
            lambda = obj.alpha^2 * (n + obj.kappa) - n;
            
            % 生成Sigma点
            sqrtP = chol((n + lambda) * obj.P, 'lower');
            X = repmat(obj.x, 1, 2*n+1);
            X(:, 2:n+1) = X(:, 2:n+1) + sqrtP;
            X(:, n+2:end) = X(:, n+2:end) - sqrtP;
            
            % 测量函数 H
            H = zeros(6, 9);
            H(1:6, 1:6) = eye(6);
            
            % 传播到测量空间
            Z = zeros(m, size(X, 2));
            for i = 1:size(X, 2)
                Z(:, i) = H * X(:, i);
            end
            
            % 计算权重
            Wm = ones(1, 2*n+1) / (2*(n+lambda));
            Wc = Wm;
            Wm(1) = lambda / (n + lambda);
            Wc(1) = lambda / (n + lambda) + (1 - obj.alpha^2 + obj.beta);
            
            % 预测测量均值
            z_pred = Z * Wm';
            
            % Innovation
            y = z - z_pred;
            innov = norm(y);
            obj.innovation_history(end+1) = innov;
            if length(obj.innovation_history) > 15
                obj.innovation_history(1) = [];
            end
            
            % 测量协方差
            Pzz = obj.R;
            for i = 1:size(Z, 2)
                diff = Z(:, i) - z_pred;
                Pzz = Pzz + Wc(i) * (diff * diff');
            end
            
            % 交叉协方差
            Pxz = zeros(n, m);
            for i = 1:size(X, 2)
                diff_x = X(:, i) - obj.x;
                diff_z = Z(:, i) - z_pred;
                Pxz = Pxz + Wc(i) * (diff_x * diff_z');
            end
            
            % Kalman增益
            K = Pxz / Pzz;
            
            % 状态更新
            obj.x = obj.x + K * y;
            
            % 协方差更新
            obj.P = obj.P - K * Pzz * K';
        end
    end
end
