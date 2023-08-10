function tests = test_arm_segment_3d
    tests = functiontests(localfunctions);
end

function setup(testCase)
    rho = 0.02;
    l_0 = 0.5;
    
    g_o = Pose3.hat(eul2rotm([0, pi/2, 0], "xyz"), [0; 0; 0]);

    testCase.TestData.l_0 = l_0;
    testCase.TestData.arm_segment = ArmSegmentFactory.make_3d_circular(4, rho, l_0, g_o);
end

function teardown(testCase)
    delete(testCase.TestData.arm_segment);
end

function test_construction(testCase)
    arm_segment = testCase.TestData.arm_segment;

    verifyEqual(testCase, length(arm_segment.rods), 4);
    verifyEqual(testCase, arm_segment.g_circ_right(1), testCase.TestData.l_0);
end

function test_set_g_circ_right(testCase)
    arm_segment = testCase.TestData.arm_segment;

    g_circ_right = [testCase.TestData.l_0; 0; 0; 0; 0; 0.3];
    arm_segment.g_circ_right = g_circ_right;

    verifyEqual(testCase, arm_segment.g_circ_right, g_circ_right);
end

function test_plotting(testCase)
    arm_segment = testCase.TestData.arm_segment;

    g_circ_right = [testCase.TestData.l_0; 0; 0; 0; 0; 0.3];
    arm_segment.g_circ_right = g_circ_right;
    
    Plotter3D.plot_arm_segment(arm_segment, axes(figure()))
    grid on
    view(30, 30)
end