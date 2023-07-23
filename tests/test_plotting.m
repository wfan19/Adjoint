function tests = test_plotting()
    tests = functiontests(localfunctions);
end

function setup(testCase)
    % Make rods
    % TODO: This is copypasted from test_rod_segment - should we move that
    % to a standalone file for reuse?
    l_0 = 1;
    testCase.TestData.l_0 = l_0;

    testCase.TestData.rod2d = RodSegment(Pose2, l_0);
    testCase.TestData.rod3d = RodSegment(Pose3, l_0);

    g_0_2d = Pose2.hat([-1; 2; -pi/2]);
    g_0_3d = Pose3.hat(eul2rotm([0, -pi/2, 0], "xyz"), [-1; -2; -3]);

    testCase.TestData.rod2d.g_0 = g_0_2d;
    testCase.TestData.rod3d.g_0 = g_0_3d;

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

    testCase.TestData.arm_segment_2d = arm_segment_2d;
    testCase.TestData.arm_series_2d = arm_series_2d;
end

function teardown(testCase)
    delete(testCase.TestData.rod2d);
    delete(testCase.TestData.rod3d);
end

function test_2d_plot_2d_rod(testCase)
    testCase.TestData.rod2d.g_circ_right = [1;0; -pi/2];
    Plotter2D.plot_rod(testCase.TestData.rod2d, axes(figure()));
end

function test_2d_plot_2d_arm_segment(testCase)
    arm_segment = testCase.TestData.arm_segment_2d;
    arm_segment.g_circ_right = [1; 0; -pi/2];

    Plotter2D.plot_arm_segment(arm_segment, axes(figure()));
end

function test_2d_plot_2d_arm_series(testCase)
    arm_series = testCase.TestData.arm_series_2d;
    arm_series.g_circ_right = [
        1, 1, 1;
        0, 0, 0;
        2, 0, -1
    ];

    Plotter2D.plot_arm_series(arm_series, axes(figure()));
    grid on
end
