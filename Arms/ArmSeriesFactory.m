classdef ArmSeriesFactory
    %ARMFACTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = ArmFactory(inputArg1,inputArg2)
            %ARMFACTORY Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
    end
        
    methods(Static)
        %% 2D
        %%% Constant radius arms
        % Constant radius 2-muscle planar arm
        % An arm with constant radius is the same as a tapered arm with no taper
        function arm_series = constant_2d_muscle_arm(N_segments, rho, l_0)
            arm_series = ArmSeriesFactory.tapered_2d_muscle_arm(N_segments, rho, rho, l_0);
        end

        % Constant radius 2-muscle antagonistic arm
        function arm_series = constant_2d_antagonist_arm(N_segments, rho_inner_base, rho_outer_base, l_0)
            arm_series = ArmSeriesFactory.tapered_2d_antagonist_arm( ...
                N_segments, rho_inner_base, rho_outer_base, rho_inner_base, rho_outer_base, l_0 ...
            );
        end

        %%% Tapered arms
        function arm_series = tapered_2d_muscle_arm(N_segments, rho_base, rho_tip, l_0)
            radii = linspace(rho_base, rho_tip, N_segments);

            segments = ArmSegment.empty(0, N_segments);
            for i = 1 : N_segments
                segments(i) = ArmSegmentFactory.make_2d_2muscle(radii(i), l_0);
            end

            arm_series = ArmSeries(segments);
        end

        function arm_series = tapered_2d_antagonist_arm(N_segments, rho_inner_base, rho_outer_base, rho_inner_tip, rho_outer_tip, l_0)
            radii_inner = linspace(rho_inner_base, rho_inner_tip, N_segments);
            radii_outer = linspace(rho_outer_base, rho_outer_tip, N_segments);

            arm_series = ArmSeriesFactory.varying_taper_2d_antagonist_arm(radii_inner, radii_outer, l_0);
        end

        function arm_series = varying_taper_2d_antagonist_arm(radii_inner, radii_outer, l_0)
            assert(length(radii_inner) == length(radii_outer), "Number of outer and inner radii do not match");
            N_segments = length(radii_inner);

            segments = ArmSegment.empty(0, N_segments);
            for i = 1 : N_segments
                segments(i) = ArmSegmentFactory.make_2d_antagonism(radii_inner(i), radii_outer(i), l_0);
            end

            arm_series = ArmSeries(segments);
        end

        %% 3D
        function arm_series = tapered_3d_muscle_arm(N_segments, rho_base, rho_tip, l_0)
            error("Not yet implemented")
        end
    end
end

