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
        l = 1    % Length of muscle
        true_shear           % Shear of muscle
        true_curvature       % Curvature of muscle

        % Components - TODO: Should these be mixins instead?
        mechanics = RodMechanicsBase()       % Mechanics model
        plotter         % Plotting module: 2D or 3D
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = RodSegment(group, l, g_0)
            arguments
                group
                l
                g_0 = -1
            end
            
            if g_0 == -1
                g_0 = eye(group.mat_size);
            end

            obj.group = group;
            obj.g_0 = g_0;

            %%% Populate default values based on the group dimensionality
            mat_e = zeros(group.algebra.mat_size);
            e_translation = group.algebra.translation(mat_e);
            e_rotation = group.algebra.rotation(mat_e);
            obj.true_shear = e_translation(2:end);
            obj.true_curvature = e_rotation;

            obj.l = l;
            obj.mechanics.l_0 = l;
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
        
        %% Setters
        % Here we implement custom setter functions to link the h_tilde
        % property with the l and kappa properties, such that updating one
        % updates the others.
        
        % Setters for l and kappa 
        function set.l(obj, l)
            assert(l ~= 0, "Length of rod cannot be zero")
            obj.l = l;
            % Update h_tilde accordingly
            obj.g_circ_right = obj.l * [1; obj.true_shear; obj.true_curvature];
        end
        
        function set.true_curvature(obj, curvature)
            obj.true_curvature = curvature;
            % Update h_tilde accordingly
            obj.g_circ_right = obj.l * [1; obj.true_shear; obj.true_curvature];
        end
        
        % Function that updates kappa and l values based on an updated
        % h_tilde value
        function set.g_circ_right(obj, g_circ_right)
            assert(g_circ_right(1) ~= 0, "Length in g_circ_right cannot be zero") % THis breaks the safeguards
            obj.g_circ_right = g_circ_right;
            
            % Update l
            new_l = g_circ_right(1);
            if obj.l ~= new_l  % Only set new val if not already set to prevent infinite loop
                obj.l = new_l;
            end

            v = obj.group.algebra.v_translation(g_circ_right);
            omega = obj.group.algebra.v_rotation(g_circ_right);

            new_shear = v(2:end) / new_l;
            new_curvature = omega / new_l;

            % Only set new val if not already set to prevent infinite loop
            if any(obj.true_shear ~= new_shear) 
                obj.true_shear = new_shear;
            end
            
            if any(obj.true_curvature ~= new_curvature)
                obj.true_curvature = new_curvature;
            end
        end
    end
end

