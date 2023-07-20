classdef se2 < gl_n
    %SE2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        dof = 3
        mat_size = [3, 3]
    end
    
    methods
        function obj = se2()
        end
    end
    
    methods (Static)
        function se2_out = hat(v_se2)
            mat_skew_sym = so2.hat(v_se2(3));
            se2_out = zeros(3, class(v_se2));
            se2_out(1:2, 1:2) = mat_skew_sym;
            se2_out(1:2, 3) = v_se2(1:2);
        end
        
        function v_se2_out = vee(se2_in)
            v_se2_out = zeros(3, 1, class(se2_in));

            v_se2_out(1:2) = se2_in(1:2, 3);
            v_se2_out(3) = se2_in(2, 1);
        end
        
        function SE2_out = expm(v_se2)
            % Analytic SE2 exponential map
            % - Supports both numerical and symbolic calculations
            % - If any input contains a symbolic object it will return a symbolic
            % matrix
            
            % Check if input was a matrix
            if numel(v_se2) ~= 3
                v_se2 = se2.vee(v_se2);
            end
            
            vx = v_se2(1);
            vy = v_se2(2);
            omega = v_se2(3);

            skew_sym_1 = so2.hat(1);
            if omega ~= 0
                V = (1/omega) * (sin(omega)*eye(2) + ((1-cos(omega))*skew_sym_1));
            else
                V = eye(2, class(v_se2));
            end

            R = SO2.hat(omega);

            SE2_out = eye(3, class(v_se2));
            SE2_out(1:2, 1:2) = R;
            SE2_out(1:2, 3) = V * [vx; vy];
        end

        function v_out = translation(se2_in)
            v_out = se2_in(1:2, 3);
        end

        function omega_out = rotation(se2_in)
            omega_out = se2_in(2, 1);
        end

        function v_out = v_translation(v_se2_in)
            v_out = v_se2_in(1:2);
            v_out = v_out(:);
        end

        function omega_out = v_rotation(v_se2_in)
            omega_out = v_se2_in(3);
            omega_out = omega_out(:);
        end
    end
end

