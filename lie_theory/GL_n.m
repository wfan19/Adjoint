classdef GL_n
    % GL_n: The general linear group
    % This absrtact class defines the interfaces that all Lie Group
    % subclasses will have
    
    properties(Abstract, Constant)
        algebra
        
        dof
        mat_size
    end
    
    methods
        function obj = GL_n()
        end
    end
    
    methods(Static, Abstract)
        hat()
        
        vee()
        
        logm()
        
        adjoint()
    end
end

