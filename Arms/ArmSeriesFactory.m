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
        function arm_series = constant_2d_muscle_arm(N_segments, rho, l_0)
            % An arm with constant radius is the same as a tapered arm with
            % no taper
            arm_series = ArmSeriesFactory.tapered_2d_muscle_arm(N_segments, rho, rho, l_0);
        end

        function arm_series = tapered_2d_muscle_arm(N_segments, rho_base, rho_tip, l_0)
            radii = linspace(rho_base, rho_tip, N_segments);

            segments = ArmSegment.empty(0, N_segments);
            for i = 1 : N_segments
                segments(i) = ArmSegmentFactory.make_2d_2muscle(radii(i), l_0);
            end

            arm_series = ArmSeries(segments);
        end

        function arm_series = tapered_antagonist_arm(N_segments, rho_inner_base, rho_outer_base)
            error("Not yet implemented")
        end

        function arm_series = tapered_3d_muscle_arm(N_segments, rho_base, rho_tip, l_0)
            error("Not yet implemented")
        end
    end
end

