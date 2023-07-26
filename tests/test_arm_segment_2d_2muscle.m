function tests = test_arm_segment_2d_2muscle
    tests = functiontests(localfunctions);
end

function setup(testCase)

    rho = 0.0254;
    l_0 = 0.5;
    arm_segment = ArmSegmentFactory.make_2d_2muscle(rho, l_0);

    % Define all rods to be bellows
    % TODO: There has to be a better way to do this
    for i = 1 : length(arm_segment.rods)
        arm_segment.rods(i).mechanics = BasicBellowMechanics(l_0);
    end

    testCase.TestData.l_0 = l_0; % Default length
    testCase.TestData.rho = rho; % Define inter-muscle geometry
    testCase.TestData.arm_segment = arm_segment;
end

function teardown(testCase)
    delete(testCase.TestData.arm_segment);
end

%% Construction and copying
function test_construction_2d(testCase)
    arm_segment = testCase.TestData.arm_segment;
    verifyEqual(testCase, testCase.TestData.l_0, arm_segment.rod_o.l);
end

function test_copy(testCase)
    % Create a copy arm and change one of its properties
    arm_copy = copy(testCase.TestData.arm_segment);
    arm_copy.rod_o.l = 2;

    % Verify that the property is unchanged on the original arm
    verifyEqual(testCase, testCase.TestData.arm_segment.rod_o.l, testCase.TestData.l_0);
end

%% Kinematics
% Set and retrieve the base curve
function test_set_get_base_curve(testCase)
    disp("not yet implemented")
    verifyEqual(testCase, 1, 1);
end

% Test shifting the arm base-curve starting poses to see if they shift 
% the individual ones as well  
function test_set_base_pose(testCase)
    arm = testCase.TestData.arm_segment;
    
    arm.g_0_o = Pose2.hat([2, 0, -pi/2]);

    verify_delta_g = inv(arm.g_0_o) * arm.rods(1).g_0;
    verifyEqual(testCase, verify_delta_g, arm.g_o_rods{1}, abstol=eps(1));
end

function test_get_tip_pose(testCase)
    tip_pose = testCase.TestData.arm_segment.get_tip_pose();
    verifyEqual(testCase, Pose2.vee(tip_pose), [0; -0.5; -pi/2]);
end

%% Mechanics
% Test retrieving the strains of each muscle, based on the base-curve
function test_get_strains(testCase)
    arm = testCase.TestData.arm_segment;
    g_circ_right = [1; 0; 2];
    strains = arm.get_strains(g_circ_right);

    % I don't have the exact values, so we'll just make sure it's curving
    % in the right direction.
    verifyLessThan(testCase, strains(2), strains(1))
end

% Test computing the forces of each muscle, given an actuation
% "What are the forces when the actuators are inflated, but the manipulator
% is held straight?"
function test_get_forces(testCase)
    arm = testCase.TestData.arm_segment;
    l_0 = testCase.TestData.l_0;

    pressures = [0; 20]; % kPa
    g_circ_right = [l_0; 0; 0];
    forces = arm.get_forces(pressures, g_circ_right);

    % I don't have the exact values, so we'll just make sure it's curving
    % in the right direction.
    verifyEqual(testCase, forces(1), 0);
    verifyGreaterThan(testCase, forces(2), 0)
end