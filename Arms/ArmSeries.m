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
            obj.segments = segments;
        end

        %% Setters and Getters
        function set.g_circ_right(obj, g_circ_right)
            for i = 1 : length(obj.segments)
                segment_i = obj.segments(i);
                segment_i.g_circ_right = g_circ_right(:, i);
            end
        end

        function g_circ_right = get.g_circ_right(obj)
            g_circ_right = zeros()
        end

        %% Member functions

        function reactions = calc_external_reaction(obj, g_circ_right, Q)

        end

        function reactions = calc_internal_reaction(obj, g_circ_right, pressures)

        end
        
        function outputArg = solve_equilibrium(obj)
        end
    end
end

