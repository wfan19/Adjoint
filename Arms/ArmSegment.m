classdef ArmSegment < handle & matlab.mixin.Copyable
    % A constant cross-section arm segment.
    properties
        group   % Group that all arm geometry is defined in
        rod_o   % Rod representing the base-curve
        rods    % List of rods composing the continuum arm

        % Supporting variables (could be private?
        % TODO: Refactor this into a segment_geometry object
        g_o_rods
        adjoints
        mat_A
    end

    properties(Dependent)
        g_circ_right
        g_0_o
        mechanics
    end
    
    methods
        function obj = ArmSegment(group, g_o, g_o_rods, l)
            N_rods = length(g_o_rods);

            obj.group = group;                                  % Store the embedding group
            obj.g_o_rods = g_o_rods;                            % Store the cross-section geometry

            obj.rod_o = RodSegment(group, l, g_o);              % Rod representing the base curve
            obj.rods = RodSegment.empty(0, N_rods);   % List of all actual rods
            obj.adjoints = cell(1, N_rods);           % Store the list of adjoint matrices for computation.

            % Create the rods
            for i = 1 : N_rods
                adjoint_i_o = obj.group.adjoint(inv(g_o_rods{i}));
                g_0_i = g_o * g_o_rods{i};

                obj.rods(i) = RodSegment(group, l, g_0_i);
                obj.adjoints{i} = adjoint_i_o;
            end

            % Create the A matrix: maps forces from actuators to total
            % force on arm's cross-section center
            obj.mat_A = zeros(obj.group.dof, N_rods);

            % TODO: Combine with above into one loop?
            % TODO: not every arm has mechanics - should this be
            % modularized, and not every model uses mat_A.
            % Should this be modularized?
            for i = 1 : N_rods
                g_o_rod_i = g_o_rods{i};
                e1 = zeros(obj.group.dof, 1);
                e1(1) = 1;
                obj.mat_A(:, i) = transpose(inv(Pose2.adjoint(g_o_rod_i))) * e1;
            end
        end

        %% Setters and Getters
        % Set and retrieve the twist-vector of the base-curve
        function g_circ_right = get.g_circ_right(obj)
            g_circ_right = obj.rod_o.g_circ_right;
        end

        function set.g_circ_right(obj, g_circ_right)
            obj.rod_o.g_circ_right = g_circ_right;
            
            for i = 1 : length(obj.rods)
                rod = obj.rods(i);

                % Use the adjoint to compute each actuator's twist-vector,
                % given the base-curve twist-vector. 
                % Note that adjoint_i_o = inv(adjoint_o_i) which is the adjoint inverse
                adjoint_i_o = obj.adjoints{i};
                g_circ_right_i = adjoint_i_o * g_circ_right;
                rod.g_circ_right = g_circ_right_i;
            end
        end

        % Set and retrive the starting pose of the base-curve
        function g_0_o = get.g_0_o(obj)
            g_0_o = obj.rod_o.g_0;
        end

        function set.g_0_o(obj, g_0_o)
            obj.rod_o.g_0 = g_0_o;

            for i = 1 : length(obj.rods)
                obj.rods(i).g_0 = g_0_o * obj.g_o_rods{i};
            end
        end

        % Set and retrieve the mechanics models of the rods in the segment
        function mechanics = get.mechanics(obj)
            mechanics = {obj.rods.mechanics};
            mechanics = mechanics(:);
        end

        function set.mechanics(obj, mechanics)
            for i = 1 : length(obj.rods)
                obj.rods(i).mechanics = mechanics{i};
            end
        end

        %% Member functions
        function tip_pose = get_tip_pose(obj, g_circ_right)
            arguments
                obj
                g_circ_right = obj.g_circ_right
            end
            obj.g_circ_right = g_circ_right;

            tip_pose = obj.group.hat(obj.rod_o.calc_posns());
        end

        % Compute the strains in each muscle and return the list
        function strains = get_strains(obj, g_circ_right)
            arguments
                obj
                g_circ_right = obj.g_circ_right
            end
            obj.g_circ_right = g_circ_right;

            strains = zeros(length(obj.rods), 1);
            for i = 1 : length(obj.rods)
                strains(i) = obj.rods(i).mechanics.strain;
            end
        end

        % Compute the forces in each muscle and return the list
        function forces = get_forces(obj, actuations, g_circ_right)
            arguments
                obj
                actuations
                g_circ_right = obj.g_circ_right
            end
            obj.g_circ_right = g_circ_right;

            forces = zeros(length(obj.rods), 1);
            for i = 1 : length(obj.rods)
                forces(i) = obj.rods(i).mechanics.get_force(actuations(i));
            end
        end

        function set_mechanics(obj, mechanics, i_rods)
            arguments
                obj
                mechanics
                i_rods = 0
            end
            if all(i_rods == 0, "all")
                % We are looking to set all mechanics models to specified
                for i = 1 : length(obj.rods)
                    obj.rods(i).mechanics = mechanics;
                end
            else
                % We are just setting the mechanics models of the specified
                % rods
                for i = 1 : length(i_rods)
                    i_rod = i_rods(i);          % Get current rod index
                    rod_i = obj.rods(i_rod);    % Get the current rod
                    rod_i.mechanics = mechanics;
                end
            end
        end

        %% Force calculations
        % TODO: These are copy pasted from the ArmSeries class
        % We could make ArmSeries just call these instead?

        % Compute the load induced at the base by a tip-load Q
        function reaction = calc_external_reaction(obj, Q, g_circ_right)
            arguments
                obj
                Q
                g_circ_right = obj.g_circ_right
            end
            % Update to a new base-curve if it is specified
            if any(g_circ_right ~= obj.g_circ_right, "all")
                obj.g_circ_right = g_circ_right;
            end
            
            g_tip = obj.get_tip_pose();
            g_i_tip = obj.group.algebra.expm(obj.g_circ_right);
    
            % Transform the force from a force Q in world coodrinates to be in local coordinates at the tip
            % In Ross parlance, this is a transform from a world-force to a right-force
            Q_right_undercirc_tip = obj.group.left_lifted_action(g_tip)' * Q;

            % Compute the left-force at the tip, which is the same as
            % the left-force at the base.
            % Mapping from right-force to left-force is done through
            % the dual adjoint inverse.
            Q_left_undercirc_i = inv(obj.group.adjoint(g_i_tip))' * Q_right_undercirc_tip;
            reaction = Q_left_undercirc_i;
        end

        % Compute the internal forces generated by the actuators
        function reactions = calc_internal_reaction(obj, pressures, g_circ_right)
            arguments
                obj
                pressures
                g_circ_right = obj.g_circ_right
            end
            % Update to a new base-curve if it is specified
            if any(g_circ_right ~= obj.g_circ_right, "all")
                obj.g_circ_right = g_circ_right;
            end
            % Compute the list of actuator forces in each segment
            forces_i = obj.get_forces(pressures, g_circ_right);

            % Apply the moment-arm matrix to the actuator forces
            reactions = obj.mat_A * forces_i;
        end

        % Compute the residuals of the static equilibrium equations:
        % The internal actuator forces and the forces from the external
        % loading must sum to zero for each segment.
        function mat_residuals = check_equilibrium(obj, pressures, Q, g_circ_right)
            arguments
                obj
                pressures
                Q
                g_circ_right = obj.g_circ_right
            end
            if any(g_circ_right ~= obj.g_circ_right, "all")
                obj.g_circ_right = g_circ_right;
            end

            internal_reactions = obj.calc_internal_reaction(pressures, g_circ_right);
            external_reactions = obj.calc_external_reaction(Q, g_circ_right);

            mat_residuals = internal_reactions + external_reactions;
            
            % Enforce shear-free by making the shear residuals the g_circ shear
            % This way we drive g_circ shear to zero
            % TODO: 1. make this not in coordinates, 2. make this not universal
            mat_residuals(2, :) = 10000 * g_circ_right(2);
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

