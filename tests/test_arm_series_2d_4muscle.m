function tests = test_arm_series_2d_4muscle
    tests = functiontests(localfunctions);
end

function setup(testCase)
    % Build a simple 2-muscle 2D arm
    l_0 = 0.5;
    rho = 0.0254;
    N_segments = 3;

    % Create a 4-muscle arm purely made of muscles
    arm_series_muscles = ArmSeriesFactory.constant_2d_antagonist_arm(N_segments, rho*1/3, rho, l_0);
    arm_series_muscles.set_mechanics(GinaMuscleMechanics(l_0));
    
    % Antagonist arm is a copy of the purely-muscle arm, but change the rod
    % model of actuators 1 and 4 to be bellows, rather than muscles
    arm_series_antagonist = copy(arm_series_muscles);
    arm_series_antagonist.set_mechanics(BasicPolyBellowMechanics(l_0), [1, 4]);

    % Save all values to the testCase TestData struct
    testCase.TestData.l_0 = l_0; % Default length
    testCase.TestData.rho = rho; % Define inter-muscle geometry
    testCase.TestData.arm_series_muscles = arm_series_muscles;
    testCase.TestData.arm_series_antagonist = arm_series_antagonist;
end

function teardown(testCase)
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
function test_ginas_model_antagonism_single_case_profile(testCase)
    profile on
    arm_series = testCase.TestData.arm_series_antagonist;
    fig = figure();
    
    % Scenario 1: Just actuate one of the McKibbens
    pressures = [0; 60; 0; 20];
    Q = [0; -5; 0];

    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    test_fetch_g_circ_right = arm_series.g_circ_right;
    Plotter2D.plot_arm_series(arm_series, axes(fig))
    title("5N load, P = [0; 60; 0; 20]")
    profile off
    profile viewer
end

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

    verifyEqual(testCase, all(abs(g_circ_right_eq(2, :)) < 0.001, "all"), true);
end