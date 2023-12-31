classdef ArmSeries < handle & matlab.mixin.Copyable
    %A variable-twist arm, composed of individually constant strain
    %ArmSegment objects
    
    properties
        segments
    end

    properties(Dependent)
        % Basic properties
        group
        N_segments
        N_rods

        % Kinematic properties
        % TODO: Is matrix form for these properties really the right way to go
        % about this?
        g_circ_right % A matrix that stores the g_circ_right of each component segments as a column.

        % A matrix of the rod mechanics involved
        mechanics
    end
    
    methods
        function obj = ArmSeries(segments)
            % TODO: Validate if the group of every segment is the same

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
        function group = get.group(obj)
            group = obj.segments(1).group;
        end

        function N_rods = get.N_rods(obj)
            N_rods = length(obj.segments(1).rods);
        end

        function N_segments = get.N_segments(obj)
            N_segments = length(obj.segments);
        end

        % Set the base curve twist vector
        function set.g_circ_right(obj, g_circ_right)
            assert( ...
                size(g_circ_right, 2) == length(obj.segments), ...
                "Number of twist-vectors supplied does not match number of segments in arm" ...
            )

            for i = 1 : obj.N_segments
                segment_i = obj.segments(i);
                segment_i.g_circ_right = g_circ_right(:, i);

                if i > 1
                    % Update the starting pose of each segment in the chain
                    % to correctly reflect the new twist vectors
                    segment_i.g_0_o = obj.segments(i-1).get_tip_pose();
                end
            end
        end

        % Get the base curve twist vector
        function mat_g_circ_right = get.g_circ_right(obj)
            mat_g_circ_right = zeros(obj.group.dof, obj.N_segments);
    
            for i = 1 : obj.N_segments
                mat_g_circ_right(:, i) = obj.segments(i).g_circ_right;
            end
        end

        % Get the mechanics models of each rod segment used
        function mat_mechanics = get.mechanics(obj)
            mat_mechanics = cell(obj.N_rods, obj.N_segments);
            for i = 1 : obj.N_segments
                mat_mechanics(:, i) = obj.segments(i).mechanics;
            end
        end

        %% Member functions
        %%% Setter and getter member functions
        function tip_pose = get_tip_pose(obj, g_circ_right)
            arguments
                obj
                g_circ_right = obj.g_circ_right
            end
            % Update to a new base-curve if it is specified
            if g_circ_right ~= obj.g_circ_right
                obj.g_circ_right = g_circ_right;
            end
            tip_pose = obj.segments(end).get_tip_pose();
        end

        function set_mechanics(obj, mechanics, i_rods)
            arguments
                obj
                mechanics
                i_rods = 0
            end

            for seg = 1 : obj.N_segments
                segment_i = obj.segments(seg);
                segment_i.set_mechanics(mechanics, i_rods)
            end
        end

        % Get the strains experineced by each muscle in each segment
        function mat_strains = get_strains(obj, g_circ_right)
            arguments
                obj
                g_circ_right = obj.g_circ_right;
            end
            obj.g_circ_right = g_circ_right;
            mat_strains = zeros(obj.N_rods, obj.N_segments);

            for i = 1 : obj.N_segments
                segment_i = obj.segments(i);
                mat_strains(:, i) = segment_i.get_strains();
            end
        end

        % Get the forces applied by each muscle in each segment
        function mat_forces = get_forces(obj, actuations, g_circ_right)
            arguments
                obj
                actuations
                g_circ_right = obj.g_circ_right;
            end
            obj.g_circ_right = g_circ_right;
            mat_forces = zeros(obj.N_rods, obj.N_segments);

            for i = 1 : obj.N_segments
                segment_i = obj.segments(i);
                mat_forces(:, i) = segment_i.get_forces(actuations, g_circ_right(:, i));
            end
        end
        
        %%% Mechanics member functions!
        function mat_reactions = calc_external_reaction(obj, Q, g_circ_right, options)
            arguments
                obj
                Q
                g_circ_right = obj.g_circ_right
                options.frame = "World"
            end
            % Update to a new base-curve if it is specified
            if any(g_circ_right ~= obj.g_circ_right, "all")
                obj.g_circ_right = g_circ_right;
            end

            g_tip = obj.get_tip_pose(); % Get the tip pose for later

            mat_reactions = zeros(obj.group.dof, length(obj.segments));
            for i = 1 : length(obj.segments)
                segment = obj.segments(i);
                g_i = segment.g_0_o;
                g_i_tip = inv(g_i) * g_tip;
        
                if strcmp(options.frame, "World")
                    % Transform the force from a force Q in world coodrinates to be in local coordinates at the tip
                    % In Ross parlance, this is a transform from a world-force to a right-force
                    Q_right_undercirc_tip = obj.group.left_lifted_action(g_tip)' * Q;
    
                    % Compute the left-force at the tip, which is the same as
                    % the left-force at the base.
                    % Mapping from right-force to left-force is done through
                    % the dual adjoint inverse.
                    Q_left_undercirc_i = inv(obj.group.adjoint(g_i_tip))' * Q_right_undercirc_tip;
                elseif strcmp(options.frame, "Tip")
                    % If the force is specified in the body frame, we don't
                    % need the left lifted action to transform it into the
                    % body frame. 
                    % 
                    % In that case it's just the adjoint to transform it
                    % from the tip to the current point on the body.
                    Q_left_undercirc_i = inv(obj.group.adjoint(g_i_tip))' * Q;
                else
                    error("Unrecognized frame name. Options are 'Tip' and 'World'")
                end

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
            if any(g_circ_right ~= obj.g_circ_right, "all")
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
        function mat_residuals = check_equilibrium(obj, pressures, Q, g_circ_right, options)
            arguments
                obj
                pressures
                Q
                g_circ_right = obj.g_circ_right
                options.frame="World"
            end
            if any(g_circ_right ~= obj.g_circ_right, "all")
                obj.g_circ_right = g_circ_right;
            end

            internal_reactions = obj.calc_internal_reaction(pressures, g_circ_right);
            external_reactions = obj.calc_external_reaction(Q, g_circ_right, "frame", options.frame);

            mat_residuals = internal_reactions + external_reactions;
            
            % Enforce shear-free by making the shear residuals the g_circ shear
            % This way we drive g_circ shear to zero
            % TODO: 1. make this not in coordinates, 2. make this not universal
            mat_residuals(2, :) = 10000 * g_circ_right(2, :);
        end

        function g_circ_right_eq = solve_equilibrium_gina(arm_series, pressures, Q, options)
            arguments
                arm_series
                pressures
                Q
                options.print=false
                options.frame="World"
            end
            % Curry the cost function: here we just solve for g_circ_right.
            f_check_eq = @(g_circ_right) arm_series.check_equilibrium(pressures, Q, g_circ_right, "frame", options.frame);

            % Optimizer options
            opt = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5, "MaxIterations", 4e3, "Algorithm", "levenberg-marquardt");     % Increase maximum allowed function evaluations
            if options.print
                opt = optimoptions(opt, "display", "final");
            else
                opt = optimoptions(opt, "display", "off");
            end 

            g_circ_right_0 = zeros(size(arm_series.g_circ_right));
            for i = 1 : length(arm_series.segments)
                g_circ_right_0(1, i) = arm_series.segments(i).rod_o.mechanics.l_0;
            end

            [g_circ_right_eq, residuals] = fsolve(f_check_eq, g_circ_right_0, opt);
            
            % Toggle whether to print the residuals
            if any(abs(residuals) > 0.01)
                disp("fzero residual is nonzero (> 0.01). Printing: ")
                disp(residuals)
            end

            arm_series.g_circ_right = g_circ_right_eq;
        end

        %% Boundary Value Problem formulation of the equilibrium model 
        function [mat_g_circ_right, mat_g_ucirc_left] = integrate_ivp(arm, pressures, g_ucirc_left_0)
            gs = cell(1, arm.N_segments);
            mat_g_ucirc_left = zeros(3, arm.N_segments);
            mat_g_circ_right = zeros(3, arm.N_segments);
        
            gs{1} = arm.segments(1).g_0_o;
            mat_g_ucirc_left(:, 1) = g_ucirc_left_0;
        
            for i = 1 : arm.N_segments
                segment_i = arm.segments(i);
                [g_circ_right, g_next, g_ucirc_left_next] = segment_i.integrate_step(pressures, mat_g_ucirc_left(:, i));
                
                if i < arm.N_segments
                    gs{i+1} = g_next;
                    mat_g_ucirc_left(:, i+1) = g_ucirc_left_next;
                end
                mat_g_circ_right(:, i) = g_circ_right;
            end
        end
        
        function residual = bvp_residual(arm, pressures, Q, g_ucirc_left_0)
                [gs, g_circ_right, g_ucirc_left] = integrate_ivp(arm, pressures, g_ucirc_left_0);
                g_tip = gs{end} * Twist2.expm(g_circ_right(:, end) * 1/3);
                Q_body = inv(Pose2.adjoint(Twist2.expm(g_circ_right(:, end) * 1/3)))' * Pose2.left_lifted_action(g_tip)' * Q;
                residual = g_ucirc_left(:, end) + Q_body;
        end
        
        function g_circ_right = solve_equilibrium_bvp(arm, pressures, Q)
            f_residual = @(g_ucirc_left_0) bvp_residual(arm, pressures, Q, g_ucirc_left_0);
            opt = optimoptions("fsolve", "algorithm", "levenberg-marquardt");
            g_ucirc_left_0_eq = fsolve(f_residual, [-5; 0; -0.17], opt);
        
            [~, g_circ_right, ~] = integrate_ivp(arm, pressures, g_ucirc_left_0_eq);
        end
    end

    methods(Access=protected)
        function cp = copyElement(obj)
            cp = copyElement@matlab.mixin.Copyable(obj);
            for i = 1 : obj.N_segments
                cp.segments(i) = copy(obj.segments(i));
            end
        end
    end
end

