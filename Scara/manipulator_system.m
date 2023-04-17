classdef manipulator_system < matlab.mixin.SetGet
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        % Constant system
        t_s
        g
        % Constant values pendulum 1
        b_1
        m_1
        l_1
        I_z1
        
        % Constant values pendulum 2
        b_2 
        m_2
        l_2
        I_z2
        
        % General vector of the states of the system
        q = zeros(4,1)
    end
    
    methods
        function obj = manipulator_system(L_1, L_2, values, x)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
            % Constante values pendulum 1
            obj.b_1 = L_1(1);
            obj.m_1 = L_1(2);
            obj.l_1 = L_1(3);
            obj.I_z1 = L_1(4);
            
            
            % Constant values pendulum 2
            obj.b_2 = L_2(1);
            obj.m_2 = L_2(2);
            obj.l_2 = L_2(3);
            obj.I_z2 = L_2(4);
            
            % General values of the system
            obj.g = values(1);
            obj.t_s = values(2);
            
            % States vector of the system
            obj.q(3:4) = x;
            obj.q(1:2) = obj.get_general_position();
            obj.q(5) = obj.manipulability_filter();
            
        end
        

        function J_t = Jacobian_p(obj)
            
            % Split vector fo the states
            q1 = obj.q(3);
            q2 = obj.q(4);
   
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = l2*cos(q1 + q2) + l1*cos(q1);
            J12 = l2*cos(q1 + q2);
            J21 = l2*sin(q1 + q2) + l1*sin(q1);
            J22 = l2*sin(q1 + q2);
            
            J_t = [J11, J12;...
                   J21, J22];
            
        end
        
        function J_t = get_Jacobian_p(obj, x)
            
            % Split vector fo the states
            q1 = x(3);
            q2 = x(4);

            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = l2*cos(q1 + q2) + l1*cos(q1);
            J12 = l2*cos(q1 + q2);
            J21 = l2*sin(q1 + q2) + l1*sin(q1);
            J22 = l2*sin(q1 + q2);
            
            J_t = [J11, J12;...
                J21, J22;...
                 1, 0;...
                 0, 1];
            
        end
        function J_t = get_Jacobian_control(obj, x)
            
            % Split vector fo the states
            q1 = x(3);
            q2 = x(4);
            
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Matrix generation Parts
            J11 = l2*cos(q1 + q2) + l1*cos(q1);
            J12 = l2*cos(q1 + q2);
            J21 = l2*sin(q1 + q2) + l1*sin(q1);
            J22 = l2*sin(q1 + q2);
            
            J_t = [J11, J12;...
                J21, J22];
            
        end
        function H = get_Hessian_1(obj, x)
            
            % Split vector fo the states
            q1 = x(3);
            q2 = x(4);
            
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            %% Hessian Matrix q1
            H = [[-l2*sin(q1 + q2) - l1*sin(q1), -l2*sin(q1 + q2)];...
                 [l2*cos(q1 + q2) + l1*cos(q1),  l2*cos(q1 + q2)]];
            
            
        end
        
        function H = get_Hessian_2(obj, x)
            
            % Split vector fo the states
            q1 = x(3);
            q2 = x(4);
            
            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            %% Hessian Matrix q1
            H = [[-l2*sin(q1 + q2), -l2*sin(q1 + q2)]
                 [l2*cos(q1 + q2),  l2*cos(q1 + q2)]];
            

            
        end
        
        function J_m = Jacobian_manipulability(obj, x)
            
            %% Hessian Matrix Definition
            hessian_1 = obj.get_Hessian_1(x);
            hessian_2 = obj.get_Hessian_2(x);
            
       
            %% Jacobian Matrix Definition
            J = obj.get_Jacobian_control(x);
            
            %% Aux Variables
            aux_hessian_1 = J*hessian_1';
            aux_hessian_2 = J*hessian_2';
            aux_J = inv(J*J'); 
            
            %% Vectorization of the matricex
            vec_h_1 = reshape(aux_hessian_1,[],1);
            vec_h_2 = reshape(aux_hessian_2,[],1);
            vec_J = reshape(aux_J,[],1);
            
            mani = obj.get_manipulability(x);
            %% Create each value of the matrices
            J_11 = mani* vec_h_1'*vec_J;
            J_21 = mani* vec_h_2'*vec_J;
            
            
            %% Asign values to the matrix
            
            J_m = [J_11;...
                   J_21];

        end
        
        function xp = f_model_manipulability(obj, u)
            
            
            % System matrices
            J = obj.Jacobian_manipulability();
            aux_v = obj.get_manipulability_filter();
            aux_m = obj.get_manipulability();
            aux = aux_v*-(2*(aux_m)/0.1);
            
            xp = aux*J'*u;
            
            
        end
        
        function xp = f_model_kinematics(obj, x, u)
            
            
            % System matrices Kinematics
            J = obj.get_Jacobian_p(x);
            % System matrices Manipulability
            J_m = obj.Jacobian_manipulability(x);
            aux_v = obj.get_manipulability_filter(x);
            aux_m = obj.get_manipulability(x);
            aux = aux_v*-(2*(aux_m)/0.1);
            
            mp = aux*J_m'*u;
            xp = J*u;
            
%             xp = [xp;mp];
            xp = [J;...
                  aux*J_m']*u;
        end
    
        
        function [x] = system_f_kinematics(obj, u)
            % Sample Time
            T_s = obj.t_s;
            
            % General States System
            x = obj.q;
            
            k1 = obj.f_model_kinematics(x, u);
            k2 = obj.f_model_kinematics(x + T_s/2*k1, u);
            k3 = obj.f_model_kinematics(x + T_s/2*k2, u);
            k4 = obj.f_model_kinematics(x + T_s*k3, u);
            x = x +T_s/6*(k1 +2*k2 +2*k3 +k4);
            
            % Update values system
            obj.q = x;
            
        end
        
        function x = get_values(obj)
           x = obj.q; 
           
        end
        function x = get_positions(obj)
           x = obj.q;
           x = x(3:4);
           
        end

        function x = get_general_position(obj)
            % Split vector fo the states
            q1 = obj.q(3);
            q2 = obj.q(4);

            
            % Split constat values
            b1 = obj.b_1;
            m1 = obj.m_1;
            l1 = obj.l_1;
            Iz1 = obj.I_z1;
            
            b2 = obj.b_2;
            m2 = obj.m_2;
            l2 = obj.l_2;
            Iz2 = obj.I_z2;
            
            % Get general Positions of the system
            hx = l2*sin(q1 + q2) + l1*sin(q1);
            hy = -l2*cos(q1 + q2) - l1*cos(q1);
            
            % General vector
            x = [hx;hy];
        end
        
        function aux_m = get_manipulability(obj, x)
            % Split vector fo the states
            J = obj.get_Jacobian_control(x);
            m = sqrt(det(J*J'));
            %aux_m = exp(-(m)^2/a_m);
            aux_m = m;
        end
        
        function aux_m = get_manipulability_filter(obj, x)
            % Split vector fo the states
            J = obj.get_Jacobian_control(x);
            m = sqrt(det(J*J'));
            a_m = 0.1;
            aux_m = exp(-(m)^2/a_m);
        end
        
         function aux_m = manipulability_filter(obj)
            % Split vector fo the states
            J = obj.Jacobian_p();
            m = sqrt(det(J*J'));
            a_m = 0.1;
            aux_m = exp(-(m)^2/a_m);
         end
         function m = manipulability(obj)
             % Split vector fo the states
             J = obj.Jacobian_p();
             m = sqrt(det(J*J'));
         end
         
    end
end

