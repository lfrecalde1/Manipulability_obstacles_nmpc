classdef controller < matlab.mixin.SetGet
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Controller gains
        K1
        K2
       
        % Pendulum system
        pendulum
    end
    
    methods
        function obj = controller(K1, K2, pendulum)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.K1 = K1;
            obj.K2 = K2;
            obj.pendulum = pendulum;
            
        end
        
        
        function xe = error_cartesian(obj, xd)
            % Get general position cartesian space
            x = obj.pendulum.get_general_position();
            
            % Get error Position cartesian Space
            xe = xd - x;
        end
        
        
        function u = get_inverse_kinematics(obj, x, xd, xdp)
            % Get Error Value
            
            xe = obj.error_cartesian(xd);
            
            % Get Error dot Value
            
            % Control Law
            J = obj.pendulum.get_Jacobian_control(x);
            
            u = inv(J)*(xdp + obj.K2*tanh(inv(obj.K2)*obj.K1*xe));
        end       
    end
end