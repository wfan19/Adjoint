classdef RodSegment < handle & matlab.mixin.Copyable
% General Rod class
% - Stores and manages the twist-vector
% - Coordinate free pose integration
% - Can own a mechanics model and plotter instance
% - Copy constructor

    %% Properties
    properties
        group           % Embedding Lie group

        g_0             % Pose of muscle in world frame
        max_s = 1       % Default s bound
        
        % Flow vector storage: components vs whole
        g_circ_right    % Flow vector of muscle
        
        % Components - TODO: Should these be mixins instead?
        mechanics = RodMechanicsBase()       % Mechanics model
        plotter         % Plotting module: 2D or 3D
    end

    properties(Dependent)
        l
        true_shear
        true_curvature
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = RodSegment(group, l, g_0)
            arguments
                group = Pose2
                l = 1
                g_0 = -1
            end
            
            if g_0 == -1
                g_0 = eye(group.mat_size);
            end

            obj.group = group;
            obj.g_0 = g_0;

            %%% Create default twist-vector g_circ_right
            mat_0 = zeros(group.algebra.mat_size);
            g_circ_right_default = group.algebra.vee(mat_0);
            g_circ_right_default(1) = l;
            obj.g_circ_right = g_circ_right_default;
        end

        %% Member functions
        % Calculate position(s) along the curve
        function [g_out, obj] = calc_posns(obj, g_circ_right, options)
            arguments
                obj
                g_circ_right = obj.g_circ_right; % Flow vector
                options.t = obj.max_s; % Array of points along the curve for the poses to be calculated at (0 - 1, percentage).
            end
                        
            % Update object's flow-vector if the input flow-vector isn't
            % the one currently stored
            if g_circ_right ~= obj.g_circ_right
                obj.g_circ_right = g_circ_right;
            end
            
            g_out = zeros(obj.group.dof, length(options.t));

            % Loop through points along the curve (t) and calculate the
            % pose for each point
            for i = 1 : length(options.t)
                % Calculate and save the transformation for each point
                pose_i = obj.g_0 * obj.group.algebra.expm(options.t(i) * obj.g_circ_right);
                g_out(:, i) = obj.group.vee(pose_i);
            end
        end
        
        %% Getters and Setters
        % TODO: Refactor l, true_shear, and true_curvature to be DEPENDENT
        % properties so we no longer have to do all the safeguarding
        % garbage.

        % Here we implement custom setter functions to link the h_tilde
        % property with the l and kappa properties, such that updating one
        % updates the others.

        % Get length of rod
        function l = get.l(obj)
            l = obj.g_circ_right(1);
        end

        function true_shear = get.true_shear(obj)
            translation = obj.group.algebra.v_translation(obj.g_circ_right);
            scaled_shear = translation(2:end);  % Shearing is the lateral components of the tangent frame linear vel
            true_shear = scaled_shear / obj.l;  % Divide scaled shearing by length to recover true shear
        end

        function true_curvature = get.true_curvature(obj)
            scaled_curvature = obj.group.algebra.v_rotation(obj.g_circ_right);  % Curvature is the tangent frame angular vel
            true_curvature = scaled_curvature / obj.l;                          % Divide scaled curvature by length to recover true curvature
        end
        
        % Setters for l and kappa 
        function set.l(obj, l)
            assert(l ~= 0, "Length of rod cannot be zero")
            % Update h_tilde accordingly
            obj.g_circ_right = l * [1; obj.true_shear; obj.true_curvature];
            obj.mechanics.update_strain(obj.l);
        end
        
        function set.true_shear(obj, true_shear)
            obj.g_circ_right = obj.l * [1; true_shear; obj.true_curvature];
        end

        function set.true_curvature(obj, true_curvature)
            % Update curvature accordingly
            obj.g_circ_right = obj.l * [1; obj.true_shear; true_curvature];
        end

        function set.g_circ_right(obj, g_circ_right)
            obj.g_circ_right = g_circ_right;
            obj.mechanics = obj.mechanics.update_strain(obj.l);
        end
    end
end

