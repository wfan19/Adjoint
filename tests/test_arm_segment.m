function tests = test_arm_segment
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

    testCase.TestData.l_0 = l_0; % Default length
    testCase.TestData.rho = 1 * 0.0254; % Define inter-muscle geometry
    testCase.TestData.arm_2d = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);
end

function teardown(testCase)
    delete(testCase.TestData.arm_2d);
end

%% Construction and copying
function test_construction_2d(testCase)
    arm_2d = testCase.TestData.arm_2d;
    verifyEqual(testCase, testCase.TestData.l_0, arm_2d.rod_o.l);
end

function test_copy(testCase)
    % Create a copy arm and change one of its properties
    arm_copy = copy(testCase.TestData.arm_2d);
    arm_copy.rod_o.l = 2;

    % Verify that the property is unchanged on the original arm
    verifyEqual(testCase, testCase.TestData.arm_2d.rod_o.l, testCase.TestData.l_0);
end

%% Kinematics
% Set and retrieve the base curve
function test_set_get_base_curve(testCase)
    disp("not yet implemented")
    verifyEqual(testCase, 1, 1);
end

%% Mechanics
% Test retrieving the strains of each muscle, based on the base-curve
function test_get_strains(testCase)
    arm = testCase.TestData.arm_2d;
    arm.set_base_curve([1; 0; 2]);
    strains = arm.get_strains();

    % I don't have the exact values, so we'll just make sure it's curving
    % in the right direction.
    verifyLessThan(testCase, strains(1), strains(2))
end

% Test computing the forces of each muscle, given an actuation
% "What are the forces when the actuators are inflated, but the manipulator
% is held straight?"
function test_get_forces(testCase)
    arm = testCase.TestData.arm_2d;
    l_0 = testCase.TestData.l_0;

    arm.set_base_curve([1; 0; 2]);
    for i = 1 : length(arm.rods)
        arm.rods(i).mechanics = BasicBellowMechanics(l_0);
    end
    pressures = [0; 20]; % kPa
    forces = arm.get_forces(pressures);

    % I don't have the exact values, so we'll just make sure it's curving
    % in the right direction.
    verifyEqual(testCase, forces(1), 0)
    verifyGreaterThan(testCase, forces(2), 0)
end