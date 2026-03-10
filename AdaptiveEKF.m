classdef AdaptiveEKF < handle
    properties
        dt
        x
        P
        F
        H
        Q_base
        Q
        R
        innovation_history
        adaptive = true
        SEQ_LENGTH = 15
    end

    methods
        function obj = AdaptiveEKF(dt, base_q)
            obj.dt = dt;

            obj.x = zeros(9,1);
            obj.P = eye(9) * 10;

            obj.F = eye(9);
            for i = 1:3
                obj.F(i,i+3) = dt;
                obj.F(i,i+6) = 0.5 * dt^2;
                obj.F(i+3,i+6) = dt;
            end

            obj.H = zeros(6,9);
            obj.H(1:6,1:6) = eye(6);

            obj.Q_base = eye(9) * base_q;
            obj.Q_base(7:9,7:9) = 2 * obj.Q_base(7:9,7:9);
            obj.Q = obj.Q_base;

            obj.R = eye(6) * 5;

            obj.innovation_history = [];
        end

        function predict(obj)
            obj.x = obj.F * obj.x;
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
        end

        function [x_post, innov] = update(obj, z, adaptive_factor)
            if nargin < 3
                adaptive_factor = 1.0;
            end

            y = z(:) - obj.H * obj.x;
            innov = norm(y);

            obj.innovation_history(end+1) = innov;
            if length(obj.innovation_history) > obj.SEQ_LENGTH
                obj.innovation_history(1) = [];
            end

            if obj.adaptive
                obj.Q = obj.Q_base * adaptive_factor;
            end

            S = obj.H * obj.P * obj.H' + obj.R;
            K = obj.P * obj.H' / S;

            obj.x = obj.x + K * y;
            obj.P = (eye(9) - K * obj.H) * obj.P;

            x_post = obj.x;
        end

        function seq = getInnovationSeq(obj)
            seq = obj.innovation_history(:);
            if length(seq) < obj.SEQ_LENGTH
                seq = padarray(seq, obj.SEQ_LENGTH-length(seq), 'replicate', 'pre');
            end
        end
    end
end
