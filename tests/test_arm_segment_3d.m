function tests = test_arm_segment_3d
    tests = functiontests(localfunctions);
end

function setup(testCase)
    rho = 0.02;
    l_0 = 0.5;

    g_o_A = Pose3.hat(eye(3), [0; 0; rho]);
    g_o_B = Pose3.hat(eye(3), [0; rho; 0]);
    g_o_C = Pose3.hat(eye(3), [0; 0; -rho]);
    g_o_D = Pose3.hat(eye(3), [0; -rho; 0]);
    g_o_rods = {g_o_A, g_o_B, g_o_C, g_o_D};
    
    g_o = Pose3.hat(eul2rotm([0, pi/2, 0], "xyz"), [0; 0; 0]);

    testCase.TestData.l_0 = l_0;
    testCase.TestData.arm_segment = ArmSegment(Pose3, g_o, g_o_rods, l_0);
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
end