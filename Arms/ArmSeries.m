classdef ArmSeries < handle & matlab.mixin.Copyable
    %A variable-twist arm, composed of individually constant strain
    %ArmSegment objects
    
    properties
        segments
    end

    properties(Dependent)
        % Is matrix form for these properties really the right way to go
        % about this?
        strains
        forces
        g_circ_right
    end
    
    methods
        function obj = ArmSeries(segments)
            % Validate if the group of every segment is the same

            % TODO: Should we manage creation of the segment list here?
            max_s = 1 / length(segments);
            for i = 1 : length(segments)
                segment = segments(i);
                
                segment.rod_o.max_s = max_s;
                for j = 1 : length(segment.rods)
                    segment.rods(j).max_s = max_s;
                end

                if i > 1
                    segment.g_0_o = segments(i-1).get_tip_pose();
                end
            end
            obj.segments = segments;
        end

        %% Setters and Getters
        function set.g_circ_right(obj, g_circ_right)
            assert( ...
                size(g_circ_right, 2) == length(obj.segments), ...
                "Number of twist-vectors supplied does not match number of segments in arm" ...
            )

            for i = 1 : length(obj.segments)
                segment_i = obj.segments(i);
                segment_i.g_circ_right = g_circ_right(:, i);

                if i > 1
                    segment_i.g_0_o = obj.segments(i-1).get_tip_pose();
                end
            end
        end

        function mat_g_circ_right = get.g_circ_right(obj)
            dof = obj.group.dof;
            N_segments = length(obj.segments);
            mat_g_circ_right = zeros(dof, N_segments);
    
            for i = 1 : N_segments
                mat_g_circ_right(:, i) = obj.segments(i).g_circ_right;
            end
        end

        % Get the strains experineced by each muscle in each segment
        function mat_strains = get.strains(obj)
            N_rods= length(obj.segments(1).rods);
            N_segments = length(obj.segments);
            mat_strains = zeros(N_rods, N_segments);

            for i = 1 : length(obj.segments)
                segment_i = obj.segments(i);
                mat_strains(:, i) = segment_i.strains;
            end
        end

        % Get the forces applied by each muscle in each segment
        function mat_forces = get.forces(obj)
            N_rods= length(obj.segments(1).rods);
            N_segments = length(obj.segments);
            mat_forces = zeros(N_rods, N_segments);

            for i = 1 : length(obj.segments)
                segment_i = obj.segments(i);
                mat_forces(:, i) = segment_i.strains;
            end
        end

        %% Member functions
        function mat_reactions = calc_external_reaction(obj, Q, g_circ_right)
            arguments
                obj
                Q
                g_circ_right = obj.g_circ_right
            end
            % Update to a new base-curve if it is specified
            if g_circ_right ~= obj.g_circ_right
                obj.g_circ_right = g_circ_right;
            end

            g_tip = obj.tip_pose(); % Get the tip pose for later
            group = obj.group;

            mat_reactions = zeros(obj.group.dof, length(obj.segments));
            for i = 1 : N_segments
                segment = obj.segments(i);
                g_i = segment.g_o;
                g_i_tip = inv(g_i) * g_tip;
        
                % Transform the force from a force Q in world coodrinates to be in local coordinates at the tip
                % In Ross parlance, this is a transform from a world-force to a right-force
                Q_right_undercirc_tip = group.left_lifted_action(g_tip)' * Q;

                % Compute the left-force at the tip, which is the same as
                % the left-force at the base.
                % Mapping from right-force to left-force is done through
                % the dual adjoint inverse.
                Q_left_undercirc_i = inv(group.adjoint(g_i_tip))' * Q_right_undercirc_tip;
                mat_reactions(:, i) = Q_left_undercirc_i;
            end
        end

        % Compute the internal forces applied by the actuators
        function mat_reactions = calc_internal_reaction(obj, pressures, g_circ_right)
            arguments
                obj
                pressures
                g_circ_right = obj.g_circ_right
            end
            % Update to a new base-curve if it is specified
            if g_circ_right ~= obj.g_circ_right
                obj.g_circ_right = g_circ_right;
            end

            mat_reactions = zeros(obj.group.dof, length(obj.segments));

            for i = 1 : length(obj.segments)
                % Compute the list of actuator forces in each segment
                segment_i = obj.segments(i);
                forces_i = segment_i.get_forces(pressures);

                % Apply the moment-arm matrix to the actuator forces
                mat_reactions(:, i) = segment_i.mat_A * forces_i;
            end
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
            if g_circ_right ~= obj.g_circ_right
                obj.g_circ_right = g_circ_right;
            end

            internal_reactions = obj.calc_internal_reaction(pressures, g_circ_right);
            external_reactions = obj.calc_external_reaction(Q, g_circ_right);

            mat_residuals = internal_reactions + external_reactions;
            
            % Enforce shear-free by making the shear residuals the g_circ shear
            % This way we drive g_circ shear to zero
            % TODO: 1. make this not in coordinates, 2. make this not universal
            mat_residuals(2, :) = g_circ_right(2, :);
        end

        function g_circ_right_eq = solve_equilibrium_gina(arm_series, pressures, Q)        
            f_check_eq = @(g_circ_right) arm_series.check_equilibrium(pressures, Q, g_circ_right);
            options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);

            [g_circ_right_eq, residuals] = fsolve(f_check_eq, arm_series.g_circ_right, options);
            
            % Toggle whether to print the residuals
            if any(residuals > 0.01)
                disp("Nonzero residual detected. Printing: ")
                disp(residuals)
            end
        end
    end
end

