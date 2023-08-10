classdef Plotter3D
    %PLOTTER3D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = Plotter3D()
        end
    end

    methods(Static)
        function plot_rod(rod, ax)
            resolution = 20;
            t = linspace(0, rod.max_s, resolution);
            v_poses = rod.calc_posns(t=t);
            position = rod.group.v_translation(v_poses);

            plot3(ax, position(1, :), position(2, :), position(3, :));
            axis(ax, "equal")
        end

        function plot_arm_segment(arm_segment, ax)
            hold(ax, "on")
            
            for i = 1 : length(arm_segment.rods)
                rod_i = arm_segment.rods(i);
                Plotter3D.plot_rod(rod_i, ax);
            end

            % Make a circle that encompases the furthest rod
            radii = vecnorm(arm_segment.mat_A(4:end, :));
            max_r = max(radii);

            function plot_circle(r, g_circle)
                t = linspace(0, 2*pi, 20);
                circle_points_body = [zeros(size(t)); cos(t); sin(t)] * r;
                circle_points_world = g_circle(1:3, 1:3) * circle_points_body + g_circle(1:3, 4);
                plot3(ax, circle_points_world(1, :), circle_points_world(2, :), circle_points_world(3, :), 'k');
            end
            plot_circle(max_r, arm_segment.g_0_o)
            plot_circle(max_r, arm_segment.get_tip_pose())
        end
    end
end

