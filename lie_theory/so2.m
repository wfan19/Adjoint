classdef so2 < gl_n
    properties (Constant)
        dof = 1
        mat_size = [2, 2]
    end
    
    methods
        function obj = so2()
        end
    end
    
    methods (Static)
        function so2_out= hat(theta)
            so2_out = [0, -theta; theta, 0];
        end
        
        function theta = vee(mat_so2)
            theta = mat_so2(2, 1);
        end
        
        function out = expm(mat_in)
            theta = mat_in(2, 1);
            out = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end
    end
end

