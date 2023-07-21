function tests = test_arm_series
    tests = functiontests(localfunctions);
end

function setup(testCase)
    % Build a simple 2-muscle 2D arm
    l_0 = 0.5;
    rho = 0.0254;
    g_o_A = Pose2.hat([0, rho, 0]);
    g_o_B = Pose2.hat([0, -rho, 0]);
    g_o_rods = {g_o_A; g_o_B};
    
    g_0_o = Pose2.hat([0, 0, -pi/2]);

    arm_segment_2d = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);

    % TODO: There should be a better way of setting muscle mechanics
    for i = 1 : length(arm_segment_2d.rods)
        arm_segment_2d.rods(i).mechanics = BasicBellowMechanics(l_0);
    end

    % TODO: Better way to create a series of segments.
    arm_segments_2d = [copy(arm_segment_2d), copy(arm_segment_2d), copy(arm_segment_2d)];
    arm_series_2d = ArmSeries(arm_segments_2d);

    % Save all values to the testCase TestData struct
    testCase.TestData.l_0 = l_0; % Default length
    testCase.TestData.rho = rho; % Define inter-muscle geometry
    testCase.TestData.arm_segment_2d = arm_segment_2d;
    testCase.TestData.arm_series_2d = arm_series_2d;
end

function teardown(testCase)
    delete(testCase.TestData.arm_segment_2d);
    delete(testCase.TestData.arm_series_2d);
end

%% Test construction and copy
function test_constructor(testCase)
    arm_series = testCase.TestData.arm_series_2d;
    l_0 = testCase.TestData.l_0;

    % Verify that there are three segments
    verifyEqual(testCase, length(arm_series.segments), 3);

    % Verify the base-curve of each segment is in its neutral
    % configuration.
    for i = 1 : length(arm_series.segments)
        segment_i = arm_series.segments(i);
        verifyEqual(testCase, segment_i.g_circ_right, [l_0; 0; 0]);
    end
end