classdef SE3 < GL_n
    %SE2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        algebra = se3
        dof = 6
        mat_size = [4, 4]
    end
    
    methods
        function obj = SE3()
        end
    end
    
    methods(Static)
        %% SE3 matrix constructor
        function SE3_out = hat(R, t)
            if class(R) ~= "sym"
                R = round(R, 5);
            end

            SE3_out = eye(4, class([R, t(:)]));

            SE3_out(1:3, 1:3) = R;
            SE3_out(1:3, 4) = t;
        end
        
        function v_SE3_out = vee(mat_SE3)
            t = mat_SE3(1:3, 4);
            R = mat_SE3(1:3, 1:3);
            eul = rotm2eul(R,"xyz");
            v_SE3_out = [t(:); eul(:)];
        end
        
        %% SE3 Logm
        function out = logm(mat_in)
            out = logm(double(mat_in));
        end
        
        %% SE3 Adjoint Action: maps between adjoint velocities
        function ad_out = adjoint(R, t)
            % Construct Adjoint matrix for SE3 based on the analytic formula
            % Reference: https://arxiv.org/pdf/1812.01537.pdf

            % The adjoint matrix maps an element from a non-identity tangent space to
            % the equivalent element in the tangent space about the identity (the Lie
            % algebra)
            
            if nargin == 1
                % Extract Rotation matrix and translation vector from single pose matrix input
                g = double(R);
                R = g(1:3, 1:3);
                t = g(1:3, 4);
            end

            ad_out = blkdiag(R, R);
            ad_out(1:3, 4:6) = so3.hat(t) * R;
        end

        function t_out = translation(SE3_in)
            t_out = SE3_in(1:3, 4);
        end

        function R_out = rotation(SE3_in)
            R_out = SE3_in(1:3, 1:3);
        end
    end
end

