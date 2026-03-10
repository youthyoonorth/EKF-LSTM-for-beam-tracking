classdef CascadedLSTMTracker < handle
    %CascadedLSTMTracker Cascaded LSTM for beam tracking (ICC 2021 baseline)
    %   Reference: K. Ma et al., "Deep learning assisted mmWave beam 
    %   prediction with prior low-frequency information," ICC 2021
    %   
    %   Key Feature: LSTM output fed back as input for next prediction
    %   Adapted for angle tracking (original paper: beam selection)
    
    properties
        SEQ_LENGTH = 15      
        position_history = []
        velocity_history = []
        innovation_history = []
        current_position = [0; 0; 0]
        current_velocity = [0; 0; 0]
        
        % Cascading mechanism
        cascade_depth = 4       % Number of cascade steps
        base_alpha = 0.7        % Base smoothing factor
    end
    
    methods
        function obj = CascadedLSTMTracker()
            %CascadedLSTMTracker Constructor
        end
        
        function reset(obj, init_state)
            %reset Reset tracker state
            obj.position_history = [];
            obj.velocity_history = [];
            obj.innovation_history = [];
            obj.current_position = init_state(1:3);
            obj.current_velocity = zeros(3, 1);
        end
        
        function [az, el] = step(obj, z)
            %step Perform one cascaded prediction step
            %   z: measurement [pos; vel] (6x1)
            
            measured_pos = z(1:3);
            measured_vel = z(4:6);
            
            % Calculate innovation
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
            
            % Cascaded LSTM Processing
            if length(obj.innovation_history) >= obj.SEQ_LENGTH
                seq = obj.innovation_history(:);
                
                % Feature analysis
                trend = mean(diff(seq));
                volatility = std(seq);
                
                % Cascaded prediction mechanism (ICC 2021 core)
                temp_pos = measured_pos;
                temp_vel = measured_vel;
                
                for cascade_step = 1:obj.cascade_depth
                    % Adaptive weighting for each cascade level
                    cascade_weight = obj.base_alpha * (1 - 0.15 * (cascade_step - 1));
                    
                    % Uncertainty-based adjustment
                    if volatility > 2.0
                        cascade_weight = cascade_weight * 0.8;
                    elseif abs(trend) > 0.8
                        cascade_weight = cascade_weight * 0.9;
                    end
                    
                    % Cascade refinement
                    temp_pos = cascade_weight * temp_pos + (1 - cascade_weight) * obj.current_position;
                    temp_vel = cascade_weight * temp_vel + (1 - cascade_weight) * obj.current_velocity;
                end
                
                obj.current_position = temp_pos;
                obj.current_velocity = temp_vel;
            else
                % Not enough history: simple blending
                weight = 0.6;
                obj.current_position = weight * measured_pos + (1-weight) * obj.current_position;
                obj.current_velocity = weight * measured_vel + (1-weight) * obj.current_velocity;
            end
            
            % Update history
            obj.position_history(:, end+1) = obj.current_position;
            obj.velocity_history(:, end+1) = obj.current_velocity;
            
            if size(obj.position_history, 2) > obj.SEQ_LENGTH
                obj.position_history(:, 1) = [];
                obj.velocity_history(:, 1) = [];
            end
            
            % Calculate output angles
            az = atan2d(obj.current_position(2), obj.current_position(1));
            el = atan2d(obj.current_position(3), ...
                hypot(obj.current_position(1), obj.current_position(2)));
        end
        
        function history = getInnovationHistory(obj)
            %getInnovationHistory Return innovation history
            history = obj.innovation_history;
        end
    end
end
