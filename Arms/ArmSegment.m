classdef ArmSegment < handle & matlab.mixin.Copyable
    % A constant cross-section arm segment.
    properties
        group   % Group that all arm geometry is defined in
        rod_o   % Rod representing the base-curve
        rods    % List of rods composing the continuum arm

        % Supporting variables (could be private?)
        adjoints
    end
    
    methods
        function obj = ArmSegment(group, g_o, g_o_rods, l)
            obj.group = group;              % Store the embedding group
            obj.rod_o = RodSegment(group, l, g_o); % Rod representing the base curve
            obj.rods = RodSegment.empty(0, length(g_o_rods));  % List of all actual rods
            obj.adjoints = cell(1, length(g_o_rods));   % Store the list of adjoint matrices for computation.

            % Create the rods
            for i = 1 : length(g_o_rods)
                adjoint_i_o = obj.group.adjoint(inv(g_o_rods{i}));
                g_0_i = g_o * g_o_rods{i};

                obj.rods(i) = RodSegment(group, l, g_0_i);
                obj.adjoints{i} = adjoint_i_o;
            end
        end

        % Set the base-curve twist-vector (g_circ_right)
        function set_base_curve(obj, g_circ_right)
            obj.rod_o.g_circ_right = g_circ_right;
            
            for i = 1 : length(obj.rods)
                % Use the adjoint to compute each actuator's twist-vector,
                % given the base-curve twist-vector. 
                % Note that adjoint_i_o = inv(adjoint_o_i) which is the adjoint inverse
                adjoint_i_o = obj.adjoints{i};
                g_circ_right_i = adjoint_i_o * g_circ_right;
                obj.rods(i).g_circ_right = g_circ_right_i;
            end
        end

        % Retrieve the base-curve twist-vector
        function g_circ_right = get_base_curve(obj)
            g_circ_right = obj.rod_o.g_circ_right;
        end

        % Retrieve the base-curve tip pose
        function pose = get_tip_pose(obj)
            pose = obj.group.hat(obj.rod_o.calc_posns());
        end

        % Compute the strains in each muscle and return the list
        function strains = get_strains(obj, g_circ_right)
            arguments
                obj
                g_circ_right = obj.get_base_curve()
            end
            obj.set_base_curve(g_circ_right);
            strains = zeros(length(obj.rods), 1);
            for i = 1 : length(obj.rods)
                rod = obj.rods(i);
                strains(i) = (rod.l - rod.mechanics.l_0) / rod.mechanics.l_0;
            end
        end
    end

    methods(Access = protected)
        function cp = copyElement(obj)
            cp = copyElement@matlab.mixin.Copyable(obj);

            % Deepcopy all the rod objects
            for i = 1 : length(obj.rods)
                cp.rods(i) = copy(obj.rods(i));
            end
            cp.rod_o = copy(obj.rod_o);
        end
    end
end

