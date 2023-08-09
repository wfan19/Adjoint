function tests = test_arm_series_2d_2muscle
    tests = functiontests(localfunctions);
end

function setup(testCase)
    % Build a simple 2-muscle 2D arm
    l_0 = 0.5;
    rho = 0.0254;
    N_segments = 3;

    arm_series_2d = ArmSeriesFactory.constant_2d_muscle_arm(N_segments, rho, l_0);
    arm_series_2d.set_mechanics(GinaMuscleMechanics(l_0));
    arm_segment_2d = arm_series_2d.segments(1);

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

%% Test force calculations under loading and actuation
function test_external_reactions(testCase)
    arm_series = testCase.TestData.arm_series_2d;
    
    Q = [0; -1; 0];
    g_circ_right = [
        0.5, 0.5, 0.5;
        0, 0, 0;
        1, 1, 1;
    ];
    
    mat_reactions = arm_series.calc_external_reaction(Q, g_circ_right);
    % TODO: Is there a validation that we want to add here, besides the
    % fact that the code runs?
end

function test_internal_reactions(testCase)
    arm_series = testCase.TestData.arm_series_2d;

    pressures = [0; 30];
    g_circ_right = [
        0.5, 0.5, 0.5;
        0, 0, 0;
        1, 1, 1;
    ];

    mat_reactions = arm_series.calc_internal_reaction(pressures, g_circ_right);
end

% Verify that at equilibrium, the force/mometn residual function is truly
% all zero.
function test_check_equilibrium(testCase)
    arm_series = testCase.TestData.arm_series_2d;
    l_0 = testCase.TestData.l_0;

    pressures_eq = [0; 0];
    Q_eq = [0; 0; 0];
    g_circ_right_eq = l_0 * [
        1 1 1;
        0 0 0;
        0 0 0
    ];

    residuals = arm_series.check_equilibrium(pressures_eq, Q_eq, g_circ_right_eq);

    verifyEqual(testCase, residuals, zeros(size(residuals)))
end

%% Test implementation of Gina's equilibrium model:
function test_ginas_model_unloaded(testCase)
    arm_series = testCase.TestData.arm_series_2d;
    Q = [0; 0; 0];
    pressures = [0; 30];
    
    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    test_fetch_g_circ_right = arm_series.g_circ_right
    
    Plotter2D.plot_arm_series(arm_series, axes(figure()));
end

function test_ginas_model_loaded(testCase)
    arm_series = testCase.TestData.arm_series_2d;
    Q = [0; -5; 0];
    pressures = [0; 30];
    
    g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
    
    Plotter2D.plot_arm_series(arm_series, axes(figure()));
end