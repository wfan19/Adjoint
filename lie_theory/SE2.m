classdef SE2 < GL_n
    %SE2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Constant)
        algebra = se2
        dof = 3
        mat_size = [3, 3]
    end
    
    methods
        function obj = SE2()
        end
    end
    
    methods(Static)
        function SE2_out = hat(v_SE2)
            x = v_SE2(1);
            y = v_SE2(2);
            theta = v_SE2(3);

            SE2_out = eye(3, class([x, y, theta]));
            SE2_out(1:2, 1:2) = SO2.hat(theta);
            SE2_out(1:2, 3) = [x; y];
        end
        
        function v_SE2_out = vee(SE2_in)
            v_SE2_out = zeros(3, 1);
            v_SE2_out(1:2) = SE2_in(1:2, 3);
            v_SE2_out(3) = atan2(SE2_in(2, 1), SE2_in(1, 1));
        end

        
        function se2_out = logm(mat_in)
            se2_out = se2(logm(mat_in));
        end
        
        function adj_out = adjoint(SE2_in)
            R = SE2_in(1:2, 1:2);
            t = SE2_in(1:2, 3);
            
            adj_out = eye(3, class([t(:); R(:)]));
            adj_out(1:2, 1:2) = R;
            adj_out(1:2, 3) = -so2.hat(1) * t(:);
        end

        function t_out = translation(SE2_in)
            t_out = SE2_in(1:2, 3);
        end

        function R_out = rotation(SE2_in)
            R_out = SE2_in(1:2, 1:2);
        end

        function TeLg = left_lifted_action(SE2_in)
            % Left lifted action on SE2
            % AKA the jacobian of a left-action delta_g * g evaluated at g
            % Formula taken from Geometric Mechanics ver 2022/12/4 pg 151
            TeLg = eye(3);
            TeLg(1:2, 1:2) = SE2_in(1:2, 1:2);
        end

        function TeRg = right_lifted_action(SE2_in)
            % Right lifted action on SE2
            % Aka the jacobian of a right-action g * delta_g evaluated at g
            % Formula taken from GM ver 2022/12/4 pg 167
            TeRg = eye(3);
            TeRg(1, 3) = -SE2_in(2, 3);
            TeRg(2, 3) = SE2_in(1, 3);
        end
    end
end

