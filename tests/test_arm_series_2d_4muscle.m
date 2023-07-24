function tests = test_arm_series_2d_4muscle
    tests = functiontests(localfunctions);
end

function setup(testCase)
    % Build a simple 2-muscle 2D arm
    l_0 = 0.5;
    rho = 0.0254;

    g_o_X = Pose2.hat([0, rho, 0]);
    g_o_A = Pose2.hat([0, rho * 1/3, 0]);
    g_o_B = Pose2.hat([0, -rho * 1/3, 0]);
    g_o_Y = Pose2.hat([0, -rho, 0]);
    g_o_rods = {g_o_X; g_o_A; g_o_B; g_o_Y};
    
    g_0_o = Pose2.hat([0, 0, -pi/2]);

    arm_segment_muscles = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);

    arm_segment_muscles.rod_o.mechanics.l_0 = l_0; % Default length of the whole segment

    % TODO: There should be a better way of setting muscle mechanics
    arm_segment_muscles.rods(1).mechanics = GinaMuscleMechanics(l_0);
    arm_segment_muscles.rods(2).mechanics = GinaMuscleMechanics(l_0);
    arm_segment_muscles.rods(3).mechanics = GinaMuscleMechanics(l_0);
    arm_segment_muscles.rods(4).mechanics = GinaMuscleMechanics(l_0);

    arm_segment_antagonist = copy(arm_segment_muscles);
    arm_segment_antagonist.rods(1).mechanics = BasicPolyBellowMechanics(l_0);
    arm_segment_antagonist.rods(4).mechanics = BasicPolyBellowMechanics(l_0);

    % TODO: Better way to create a series of segments.
    N_segments=  3;
    arm_segments_muscles = ArmSegment.empty(0, N_segments);
    for i = 1 : N_segments
        arm_segments_muscles(i) = copy(arm_segment_muscles);
    end
    arm_series_muscles = ArmSeries(arm_segments_muscles);

    arm_segments_antagonist= ArmSegment.empty(0, N_segments);
    for i = 1 : N_segments
        arm_segments_antagonist(i) = copy(arm_segment_antagonist);
    end
    arm_series_antagonist = ArmSeries(arm_segments_antagonist);

    % Save all values to the testCase TestData struct
    testCase.TestData.l_0 = l_0; % Default length
    testCase.TestData.rho = rho; % Define inter-muscle geometry
    testCase.TestData.arm_segment_muscles = arm_segment_muscles;
    testCase.TestData.arm_series_muscles = arm_series_muscles;

    testCase.TestData.arm_series_antagonist = arm_series_antagonist;
end

function teardown(testCase)
    delete(testCase.TestData.arm_segment_muscles);
    delete(testCase.TestData.arm_series_muscles);
end
    
%% Test constructing and plotting the arm
function test_construction(testCase)
    arm_series = testCase.TestData.arm_series_muscles;
    verifyEqual(testCase, length(arm_series.segments), 3)
    verifyEqual(testCase, length(arm_series.segments(1).rods), 4)
end

function test_plotting(testCase)
    arm_series = testCase.TestData.arm_series_muscles;
    Plotter2D.plot_arm_series(arm_series, axes(figure()))
end

%% Test solving the mechanics
function test_ginas_model_four_muscles(testCase)
    arm_series = testCase.TestData.arm_series_muscles;
    fig = figure();
    
    % Scenario 1: Just actuate one of the McKibbens
    pressures = [0; 60; 0; 0];
    Q = [0; 0; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_1 = subplot(2, 2, 1);
    Plotter2D.plot_arm_series(arm_series, ax_1)
    title("Unloaded, P = [0; 60; 0; 0]")
    
    % Scenario 2: Now activate antagonism but without load
    % This might make it bend more?
    pressures = [0; 60; 0; 10];
    Q = [0; 0; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_2 = subplot(2, 2, 2);
    Plotter2D.plot_arm_series(arm_series, ax_2)
    title("Unloaded, P = [0; 60; 0; 10]")
    
    % Scenario 3: Now apply a load, but wihout antagonism yet
    pressures = [0; 60; 0; 0];
    Q = [0; -5; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_3 = subplot(2, 2, 3);
    Plotter2D.plot_arm_series(arm_series, ax_3)
    title("5N load, P = [0; 60; 0; 0]")
    
    % Scenario 4: Antagonistic actuation with load
    % It should bend less than without antagonism!
    pressures = [0; 60; 0; 10];
    Q = [0; -5; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_4 = subplot(2, 2, 4);
    Plotter2D.plot_arm_series(arm_series, ax_4)
    title("5N load, P = [0; 60; 0; 10]")
    
    sgtitle("Muscle | Muscle | Muscle | Muscle")
end

function test_ginas_model_antagonism(testCase)
    arm_series = testCase.TestData.arm_series_antagonist;
    fig = figure();
    
    % Scenario 1: Just actuate one of the McKibbens
    pressures = [0; 60; 0; 0];
    Q = [0; 0; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_1 = subplot(2, 2, 1);
    Plotter2D.plot_arm_series(arm_series, ax_1)
    title("Unloaded, P = [0; 60; 0; 0]")
    
    % Scenario 2: Now activate antagonism but without load
    % This might make it bend more?
    pressures = [0; 60; 0; 10];
    Q = [0; 0; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_2 = subplot(2, 2, 2);
    Plotter2D.plot_arm_series(arm_series, ax_2)
    title("Unloaded, P = [0; 60; 0; 10]")
    
    % Scenario 3: Now apply a load, but wihout antagonism yet
    pressures = [0; 60; 0; 0];
    Q = [0; -5; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_3 = subplot(2, 2, 3);
    Plotter2D.plot_arm_series(arm_series, ax_3)
    title("5N load, P = [0; 60; 0; 0]")
    
    % Scenario 4: Antagonistic actuation with load
    % It should bend less than without antagonism!
    pressures = [0; 60; 0; 10];
    Q = [0; -5; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    ax_4 = subplot(2, 2, 4);
    Plotter2D.plot_arm_series(arm_series, ax_4)
    title("5N load, P = [0; 60; 0; 10]")

    sgtitle("Bellow | Muscle | Muscle | Bellow")
end