classdef MuscleMechanics < RodMechanicsBase
    
    properties
        l_0 = 1
        f_force = @f_force_default
    end
    
    methods
        function obj = MuscleMechanics(l_0, f_force)
            arguments
                l_0 = 1;
                f_force = @(strain, pressure) 0;
            end

            obj.l_0 = l_0;
            
            
        end
        
        function force = f_force_default(strain, pressure)
            force = 0;
        end
    end
end

