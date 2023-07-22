classdef RodMechanicsBase
    % Base class of any rod mechanics model
    % A rod mechanics model encompasses how an elastic rod behaves on its
    % own. Thus it may include any of the following:
    % - Neutral length
    % - Force produced as a function of strain/pressure (force-surface).
    % - Linear stiffness
    % - Bending stiffness
    % - Stiffness matrix
    
    properties
        l_0 = 1
        strain = 0
        f_force = @f_force_default
    end
    
    methods
        function obj = RodMechanicsBase(l_0, f_force)
            arguments
                l_0 = 1;
                f_force = @f_force_default;
            end

            obj.l_0 = l_0;
            obj.f_force = f_force;
        end
        
        % We assume by default that an actuator's force is entirely
        % characterized by the strain and input.
        function force = f_force_default(obj, strain, actuation)
            force = 0;
        end

        function force = get_force(obj, actuation)
            force = obj.f_force(obj.strain, actuation);
        end
        
        function obj = update_strain(obj, l)
            obj.strain = (l - obj.l_0) / obj.l_0;
        end
    end
end

