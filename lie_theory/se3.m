classdef se3 < gl_n
    %SE3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        dof = 6
        mat_size = [4, 4]
    end
    
    methods
        function obj = se3()
        end
    end
    
    methods (Static)
        %% se3 matrix constructor
        function se3_out = hat(v_se3)
            v = v_se3(1:3); 
            omega = v_se3(4:6);
            se3_out = zeros(4, class(v_se3));

            mat_omega = so3.hat(omega);

            se3_out(1:3, 1:3) = mat_omega;
            se3_out(1:3, 4) = v;            
        end
        
        %% SE3 Vee: Maps from matrix representation to vector representation
        function v_se3_out = vee(mat_se3)
            v_vel = mat_se3(1:3, 4);

            mat_omega = mat_se3(1:3, 1:3);
            v_omega = [mat_omega(3, 2);
                        -mat_omega(3, 1);
                        mat_omega(2, 1)
            ];

            v_se3_out = [v_vel; v_omega];
        end
        
        %% SE3 Expm
        function SE3_out = expm(v_twist)
            % Analytic form of the SE3 exponentail map, from here: https://arxiv.org/pdf/1812.01537.pdf
            % Extract linear/angular components from a single vector
            if size(v_twist) == se3.mat_size
                v_twist = se3.vee(v_twist);
            end
            
            vel = v_twist(1:3);
            omega = v_twist(4:6);

            omega = omega(:);
            vel = vel(:);

            class_string = class([omega(:), vel(:)]);
            SE3_out = eye(4, class_string);

            theta = norm(omega);

            if class(theta) == "sym"
                    V = eye(3, class_string) + (1 - cos(theta)) / theta^2 * so3.hat(omega) + ...
                        (theta - sin(theta))/theta^3 * so3.hat(omega)^2;
            else
                if round(theta, 6) > 0 % Separate because "round(sym) > 0" throws errors
                    V = eye(3, class_string) + (1 - cos(theta)) / theta^2 * so3.hat(omega) + ...
                        (theta - sin(theta))/theta^3 * so3.hat(omega)^2;
                else
                    % For small angles this is what the coefficients converge to, not
                    % infinity
                    % Can be calculated via l'Hopitals
                    V = eye(3, class_string) + 0.5 * so3.hat(omega) + (1/6) * so3.hat(omega);
                end 
            end

            SE3_out(1:3, 1:3) = so3.expm(omega);
            SE3_out(1:3, 4) = V * vel;
        end

        function v_out = translation(se3_in)
            v_se3_in = se3.vee(se3_in);
            v_out = se3.v_translation(v_se3_in);
        end

        function omega_out = rotation(se3_in)
            v_se3_in = se3.vee(se3_in);
            omega_out = se3.v_rotation(v_se3_in);
        end

        function v_out = v_translation(v_se3_in)
            v_out = v_se3_in(1:3);
            v_out = v_out(:);
        end

        function omega_out = v_rotation(v_se3_in)
            omega_out = v_se3_in(4:6);
            omega_out = omega_out(:);
        end
    end
end

