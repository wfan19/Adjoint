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
    end
end

