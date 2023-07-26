classdef ArmSegmentFactory
    %SEGMENTFACTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end

    methods(Static)
        function arm_segment = make_2d_2muscle(rho, l_0)
            g_o_A = Pose2.hat([0, -rho, 0]);
            g_o_B = Pose2.hat([0, rho, 0]);
            g_o_rods = {g_o_A; g_o_B};

            g_0_o = Pose2.hat([0, 0, -pi/2]);
    
            arm_segment = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);
            arm_segment.rod_o.mechanics.l_0 = l_0; % Default length of the whole segment
        end

        function arm_segment = make_2d_antagonism(rho_inner, rho_outer, l_0)
            g_o_X = Pose2.hat([0, -rho_outer, 0]);
            g_o_A = Pose2.hat([0, -rho_inner, 0]);
            g_o_B = Pose2.hat([0, rho_inner, 0]);
            g_o_Y = Pose2.hat([0, rho_outer, 0]);
            g_o_rods = {g_o_X; g_o_A; g_o_B; g_o_Y};

            g_0_o = Pose2.hat([0, 0, -pi/2]);
    
            arm_segment = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);
            arm_segment.rod_o.mechanics.l_0 = l_0; % Default length of the whole segment
        end

        function arm_segment = make_3d_circular(rho, l_0)
            % Create list of angles around the circle that the rods will sit at
            theta = linspace(0, 2*pi, N_rods + 1);
            theta = theta(1:end-1);
            
            g_o_rods = cell(1, N_rods);
            for i = 1 : n_muscles
                % Posn of each rod is based on the angle around the circle
                t_muscle_i = rho * [0; cos(theta(i)); sin(theta(i))];

                % Rotate each rod such that (Y? Z?) axis is tangent to circle
                R_muscle_i = eul2rotm([yaw_muscles(i), 0, 0], 'xyz');
                g_o_rods{i} = Pose3.hat(R_muscle_i, t_muscle_i);
            end

            g_0_o = Pose3(eul2rotm([0, -pi/2, 0], "xyz"), [0; 0; 0]);
            arm_segment = ArmSegment(Pose3, g_0_o, g_o_rods, l_0);
            
            for i = 1 : length(arm_segment.rods)
                arm_segment.rods(i).mechanics = GinaMuscleMechanics(l_0);
            end
        end
    end
end

