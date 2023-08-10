classdef Plotter2D
    %PLOTTER2D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Here's an idea: what if a plotter instance owned an arm, and the
        % line handles for the arm?
    end
    
    methods
        function obj = Plotter2D()
        end
    end
    % All plotting functions are static for now
    % Might make Plotter a proper class later?
    methods(Static)
        function plot_rod(rod, ax)
            resolution = 20;
            t = linspace(0, rod.max_s, resolution);
            v_poses = rod.calc_posns(t=t);
            position = rod.group.v_translation(v_poses);

            % Just plot x and y, even if the arm is in 3D.
            plot(ax, position(1, :), position(2, :));
            axis(ax, "equal")
        end

        function plot_arm_segment(arm_segment, ax)
            hold(ax, "on")
            
            for i = 1 : length(arm_segment.rods)
                rod_i = arm_segment.rods(i);
                Plotter2D.plot_rod(rod_i, ax);
            end
        end

        function plot_arm_series(arm_series, ax)    
            for i = 1 : length(arm_series.segments)
                segment_i = arm_series.segments(i);
                Plotter2D.plot_arm_segment(segment_i, ax);
            end
            grid(ax, "on")
        end

        function plot_g_circ_right(arm_series, parent)
            cell_g_circ_right = cell(1, arm_series.N_rods);
            for i = 1 : arm_series.N_rods
                % For each rod, collect twist of each segment
                rod_i_g_circ_right = zeros(3, arm_series.N_segments);
                for j = 1 : arm_series.N_segments
                    % Store the segment twist as column in the rod's twist matrix
                    rod_i_g_circ_right(:, j) = arm_series.segments(j).rods(i).g_circ_right;
                end
                cell_g_circ_right{i} = rod_i_g_circ_right;
            end

            % Now we plot the values for each rod
            titles = ["Length", "Shear", "Length-scaled Curvature"];
            tl = tiledlayout(parent, 1, 3);
            for i = 1 : 3
                ax = nexttile(tl);
                cla(ax);
                hold(ax, "on")
                grid(ax, "on")
                s = linspace(0, 1, arm_series.N_segments);
                for j = 1 : length(cell_g_circ_right)
                    rod_i_g_circ_right = cell_g_circ_right{j};
                    vals = rod_i_g_circ_right(i, :);
                    plot(ax, s, vals);
                end
                title(ax, titles(i));
                legend(ax, string(1:arm_series.N_rods));
            end
        end
    end
end

