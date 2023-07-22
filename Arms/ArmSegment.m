classdef ArmSegment < handle & matlab.mixin.Copyable
    % A constant cross-section arm segment.
    properties
        group   % Group that all arm geometry is defined in
        rod_o   % Rod representing the base-curve
        rods    % List of rods composing the continuum arm

        % Supporting variables (could be private?
        % TODO: Refactor this into a segment_geometry object
        adjoints
        mat_A
    end

    properties(Dependent)
        g_circ_right
    end
    
    methods
        function obj = ArmSegment(group, g_o, g_o_rods, l)
            N_rods = length(g_o_rods);

            obj.group = group;                                  % Store the embedding group
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

        function g_circ_right = get.g_circ_right(obj)
            g_circ_right = obj.rod_o.g_circ_right;
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

