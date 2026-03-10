classdef ODELSTMTracker < handle
    %ODELSTMTracker ODE-LSTM for beam tracking
    %   Based on continuous-time modeling with neural ODEs
    %   Adapted from ODE-LSTM for angle tracking
    
    properties
        SEQ_LENGTH = 15
        solver_type = 'euler'  % Options: 'euler', 'heun', 'rk4'
        num_unfolds = 3        % Number of ODE integration unfolds
        
        position_history = []
        velocity_history = []
        innovation_history = []
        current_position = [0; 0; 0]
        current_velocity = [0; 0; 0]
        
        dt_base = 0.001
        alpha = 0.6
    end
    
    methods
        function obj = ODELSTMTracker(solver_type)
            %ODELSTMTracker Constructor
            if nargin > 0
                obj.solver_type = solver_type;
            end
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
            %step Perform one ODE-LSTM step
            
            measured_pos = z(1:3);
            measured_vel = z(4:6);
            
            % Calculate innovation
            if isempty(obj.position_history)
                innovation = norm(measured_pos - obj.current_position);
            else
                predicted_pos = obj.current_position + obj.current_velocity * obj.dt_base;
                innovation = norm(measured_pos - predicted_pos);
            end
            
            obj.innovation_history(end+1) = innovation;
            if length(obj.innovation_history) > obj.SEQ_LENGTH
                obj.innovation_history(1) = [];
            end
            
            % ODE-LSTM Processing
            if length(obj.innovation_history) >= obj.SEQ_LENGTH
                % ODE Integration (continuous-time evolution)
                delta_t = obj.dt_base / obj.num_unfolds;
                temp_state = [obj.current_position; obj.current_velocity];
                
                for unfold = 1:obj.num_unfolds
                    switch obj.solver_type
                        case 'euler'
                            temp_state = obj.euler_step(temp_state, delta_t, measured_pos, measured_vel);
                        case 'heun'
                            temp_state = obj.heun_step(temp_state, delta_t, measured_pos, measured_vel);
                        case 'rk4'
                            temp_state = obj.rk4_step(temp_state, delta_t, measured_pos, measured_vel);
                        otherwise
                            temp_state = obj.euler_step(temp_state, delta_t, measured_pos, measured_vel);
                    end
                end
                
                obj.current_position = temp_state(1:3);
                obj.current_velocity = temp_state(4:6);
            else
                obj.current_position = obj.alpha * measured_pos + (1-obj.alpha) * obj.current_position;
                obj.current_velocity = obj.alpha * measured_vel + (1-obj.alpha) * obj.current_velocity;
            end
            
            % Update history
            obj.position_history(:, end+1) = obj.current_position;
            obj.velocity_history(:, end+1) = obj.current_velocity;
            
            if size(obj.position_history, 2) > obj.SEQ_LENGTH
                obj.position_history(:, 1) = [];
                obj.velocity_history(:, 1) = [];
            end
            
            % Output angles
            az = atan2d(obj.current_position(2), obj.current_position(1));
            el = atan2d(obj.current_position(3), ...
                hypot(obj.current_position(1), obj.current_position(2)));
        end
        
        %% ODE Solvers
        
        function new_state = euler_step(obj, state, dt, meas_pos, meas_vel)
            %euler_step Euler integration method
            pos = state(1:3);
            vel = state(4:6);
            
            d_pos = vel;
            d_vel = (meas_vel - vel) * 5.0 + (meas_pos - pos) * 2.0;
            
            new_pos = pos + dt * d_pos;
            new_vel = vel + dt * d_vel;
            
            new_state = [new_pos; new_vel];
        end
        
        function new_state = heun_step(obj, state, dt, meas_pos, meas_vel)
            %heun_step Heun's method (2nd order)
            pos = state(1:3);
            vel = state(4:6);
            
            % k1
            d_pos1 = vel;
            d_vel1 = (meas_vel - vel) * 5.0 + (meas_pos - pos) * 2.0;
            
            % k2
            pos_temp = pos + dt * d_pos1;
            vel_temp = vel + dt * d_vel1;
            d_pos2 = vel_temp;
            d_vel2 = (meas_vel - vel_temp) * 5.0 + (meas_pos - pos_temp) * 2.0;
            
            % Average
            new_pos = pos + dt * 0.5 * (d_pos1 + d_pos2);
            new_vel = vel + dt * 0.5 * (d_vel1 + d_vel2);
            
            new_state = [new_pos; new_vel];
        end
        
        function new_state = rk4_step(obj, state, dt, meas_pos, meas_vel)
            %rk4_step Runge-Kutta 4th order
            pos = state(1:3);
            vel = state(4:6);
            
            % k1
            d_pos1 = vel;
            d_vel1 = (meas_vel - vel) * 5.0 + (meas_pos - pos) * 2.0;
            
            % k2
            pos2 = pos + dt * 0.5 * d_pos1;
            vel2 = vel + dt * 0.5 * d_vel1;
            d_pos2 = vel2;
            d_vel2 = (meas_vel - vel2) * 5.0 + (meas_pos - pos2) * 2.0;
            
            % k3
            pos3 = pos + dt * 0.5 * d_pos2;
            vel3 = vel + dt * 0.5 * d_vel2;
            d_pos3 = vel3;
            d_vel3 = (meas_vel - vel3) * 5.0 + (meas_pos - pos3) * 2.0;
            
            % k4
            pos4 = pos + dt * d_pos3;
            vel4 = vel + dt * d_vel3;
            d_pos4 = vel4;
            d_vel4 = (meas_vel - vel4) * 5.0 + (meas_pos - pos4) * 2.0;
            
            % Weighted average
            new_pos = pos + dt * (d_pos1 + 2*d_pos2 + 2*d_pos3 + d_pos4) / 6.0;
            new_vel = vel + dt * (d_vel1 + 2*d_vel2 + 2*d_vel3 + d_vel4) / 6.0;
            
            new_state = [new_pos; new_vel];
        end
        
        function history = getInnovationHistory(obj)
            %getInnovationHistory Return innovation history
            history = obj.innovation_history;
        end
    end
end
