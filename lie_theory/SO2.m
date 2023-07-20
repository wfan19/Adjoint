classdef SO2 < GL_n
        
    properties (Constant)
        algebra = so2
        dof = 1
        mat_size = [2, 2]
    end
    
    methods
        function obj = SO2()
        end
    end
    
    methods(Static)
        function SO2_out = hat(theta)
             SO2_out = [cos(theta), -sin(theta); sin(theta), cos(theta)];
             if class(theta) ~= "sym" % Round if we are constructing a numeric matrix
                SO2_out = round(SO2_out, 8); % 8 places should be sufficient?
             end
        end
        
        function out = vee(in)
            out = 0;
        end
        
        function out = logm(mat_in)
            out = logm(mat_in);
        end
        
        function out = adjoint(in)
            out = in;
        end
    end
end