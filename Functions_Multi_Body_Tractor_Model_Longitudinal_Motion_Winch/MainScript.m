%% Joshua Thomas Cook 02/08/2016 Dartmouth College:
% MT865 Tractor-Sled Simulation

% Initialization
clear, clc, close all
addpath('/Users/joshuacook/Desktop/Final_Tractor_Models_Code/Kalman_Filter_State_Force_Estimation')
addpath('/Users/joshuacook/Desktop/Final_Tractor_Models_Code/Bayes_Filter_Terrain_Paramter_Estimation')
addpath('/Users/joshuacook/Desktop/Final_Tractor_Models_Code/ECU_Traction_Control')

%% --------- Initialize Structure of constant tractor parameters ----------
resCoeff = 0.11;
bladderNo = 8;
nConstantMT865 = initialize_constant_tractors_parameters(resCoeff,bladderNo);

%% -------------------- Initialize Terrain Map ----------------------------
% Terrain Bondries: nLocX = [minX1 maxX1; minX2 maxX2; ... ];
%                   nLocY = [minY1 maxY1; minY2 maxY2; ... ];

% --------------------------- Tractor1 ------------------------------------
nConstantTerrain.indTrac(:,1) = [1:3].';
nConstantTerrain.nLocX(1:3,1:2) = [0 30; 30 130; 130 250]; 
nConstantTerrain.nLocY(1:3,1:2) = [0 10; 0 10; 0 10];
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(1:3,1) = [6.1 3 8].'; 
nConstantTerrain.frictionAngle(1:3,1) = [20 17.8 20].';
nConstantTerrain.n(1:3,1) = [1 1 1].';
nConstantTerrain.keq(1:3,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(1:3,1) = [2 0.7 2].';
nConstantTerrain.S(1:3,1) = [0.6/33 0.8/33 0.6/33].';

% --------------------------- Tractor2 ------------------------------------
nConstantTerrain.indTrac(:,2) = [4:6].';
nConstantTerrain.nLocX(4:6,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(4:6,1:2) = nConstantTerrain.nLocY(1:3,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(4:6,1) = [6.1 3.9 6.3].'; 
nConstantTerrain.frictionAngle(4:6,1) = [20 17.1 20].';
nConstantTerrain.n(4:6,1) = [1 1 1].';
nConstantTerrain.keq(4:6,1) = [500 470 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(4:6,1) = [2 2.7 4].';
nConstantTerrain.S(4:6,1) = [0.6/33 0.9/33 0.6/33].';

% --------------------------- Tractor3 ------------------------------------
nConstantTerrain.indTrac(:,3) = [7:9].';
nConstantTerrain.nLocX(7:9,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(7:9,1:2) = nConstantTerrain.nLocY(4:6,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(7:9,1) = [6.1 4.3 6.3].'; 
nConstantTerrain.frictionAngle(7:9,1) = [20 18.15 20].';
nConstantTerrain.n(7:9,1) = [1 1 1].';
nConstantTerrain.keq(7:9,1) = [500 320 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(7:9,1) = [2 4.9 4].';
nConstantTerrain.S(7:9,1) = [0.6/33 0.8/33 0.6/33].';

% --------------------------- Tractor4 ------------------------------------
nConstantTerrain.indTrac(:,4) = [10:12].';
nConstantTerrain.nLocX(10:12,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(10:12,1:2) = nConstantTerrain.nLocY(7:9,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(10:12,1) = [6.1 5.7 6.3].'; 
nConstantTerrain.frictionAngle(10:12,1) = [20 16.8 20].';
nConstantTerrain.n(10:12,1) = [1 1 1].';
nConstantTerrain.keq(10:12,1) = [500 375 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(10:12,1) = [2 6.8 4].';
nConstantTerrain.S(10:12,1) = [0.6/33 0.9/33 0.6/33].';

% --------------------------- Tractor5 ------------------------------------
indexTerrain5 = [13:15].';
nConstantTerrain.indTrac(:,5) = [13:15].';
nConstantTerrain.nLocX(indexTerrain5,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(indexTerrain5,1:2) = nConstantTerrain.nLocY(10:12,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(indexTerrain5,1) = [6.1 6.1 6.3].'; 
nConstantTerrain.frictionAngle(indexTerrain5,1) = [20 15.3 20].';
nConstantTerrain.n(indexTerrain5,1) = [1 1 1].';
nConstantTerrain.keq(indexTerrain5,1) = [500 490 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(indexTerrain5,1) = [2 7 4].';
nConstantTerrain.S(indexTerrain5,1) = [0.6/33 0.8/33 0.6/33].';

nConstantTerrain.terrainActual = [nConstantTerrain.cohesion.'; nConstantTerrain.frictionAngle.'; ...
    nConstantTerrain.n.'; nConstantTerrain.keq.'; nConstantTerrain.K.'; nConstantTerrain.S.'];
% Grid Specifications
nConstantTerrain.gridSizeXM = 250;
nConstantTerrain.gridSizeYM = 50;
nConstantTerrain.gridResolutionM = 0.1;
nConstantTerrain.Mode = 'Region';
nConstantTerrain = generate_terrain(nConstantTerrain,nConstantMT865);

%% --------------- Set Simulation Integration Time Steps ------------------
nTimeParam.timeStepS = 0.05;
nTimeParam.simulationTime = 90;
nTimeParam.time = [0:nTimeParam.timeStepS:nTimeParam.simulationTime].'; % Time array based on sample time and total simulation time
nTimeParam.nTimeStep = size(nTimeParam.time,1); % Total number of time steps

%% ------------------- Set Open Loop Control Inputs -----------------------

% Tractor One - Do nothing with winch
throttleTractor1 =      [0.24 0.26  0.3 0.4 0.5 0.6  0.7  0.7  0.9 0.9  0.8 0.9 0.4 0.4 0.4].';
%gearTractor1 =          [2    3     4    5    6   7    8    9    9   9    9   9   9   9   9  ].';
gearTractor1 =          [2    3     4    5    6   7    8    8    8   8    8   8   8   8   8  ].';
steerAngleDegTractor1 = [0    0     0    0    0   0    0    0    0   0    0   0   0   0   0  ].'; 
clutchCmd1 =            [1    1     1    1    1   1    1    1    1   1    1   1   1   1   1  ].';
valvePosition1 =        [2    2     2    2    2   2    2    2    2   2    2   2   2   2   2  ].'; % 1 - Pull In, 2 - Brake Position 3, Let Sled pull Out Winch
pSet1 = [2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700].'*6894.76 ;% psi to N/m^2
timeScheduleInput1 =    [0    4     8   12   16   18   22   26   45  35   40  91  24  26  30.1].';
inputMat1 = [throttleTractor1 gearTractor1 steerAngleDegTractor1 clutchCmd1 valvePosition1 pSet1 timeScheduleInput1];
inputMat1 = inputMat_make( inputMat1, nTimeParam.timeStepS );


%% Initialize Kalman Filter and Bayes Estimator and Traction Control

% Kalman Filter
structDTKF_1 = intialize_DTKF([1E-3 1E-5 2E6 2E5 2E7 2E8 1E6 2E5].', nConstantMT865, nTimeParam);
controlArchitecture.structDTKF = structDTKF_1;

% Recursive Bayes Estimate
structRBE.terrainHypDefCohesion = 1:1:8;
structRBE.terrainHypDefFrictionAngle = 20;
structRBE.terrainHypDefn = 1;
structRBE.terrainHypDefkeq = 100:100:500;
structRBE.terrainHypDefK = 0.5:1:3.5;
structRBE.terrainHypDefS = 0.6/33;
structRBE.FlagRBEisOn = 1;
structRBE_1 = structRBE;
structRBE_1.covariance = [5e8 5e8];
structRBE_1.normalizeString = 'noNormalize';
structRBE_1.lowProbThreshold = 1e-3;
structRBE_1 = terrain_hypothesis(structRBE_1, nConstantMT865, nTimeParam);

structRBE_2 = structRBE;
structRBE_2.covariance = [2e-3 2e-3];
structRBE_2.normalizeString = 'normalize';
structRBE_2.lowProbThreshold = 1e-3;
structRBE_2 = terrain_hypothesis(structRBE_2, nConstantMT865, nTimeParam);

structRBE_3 = structRBE;
structRBE_3.terrainHypDefK = 0.5:1:7.5;
structRBE_3.covariance = structRBE_2.covariance;
structRBE_3.normalizeString = 'normalize';
structRBE_3.lowProbThreshold = 1e-4;
structRBE_3 = terrain_hypothesis(structRBE_3, nConstantMT865, nTimeParam);
controlArchitecture.structRBE = structRBE_3;

% Traction Controller
% Kp = 0.27, Ki = 0.15, Kd = 1E-6, Tf = 1E-6
structTractionController = initialize_TractionController( 0.28, 0.22, 0.03, 0.3, nConstantMT865, nTimeParam.nTimeStep);
%structTractionController.gearControlString = 'maxPower';
structTractionController.peakSlipRefString = 'peakSlipRefSmooth';
structTractionController.gearControlString = 'engineRPMRange';
controlArchitecture.structTractionController = structTractionController;

% Winch Controller
structWinchController = initialize_Winch_Controller(nTimeParam);
controlArchitecture.structWinchController = structWinchController;


%% ------------ Initialize Tractor and Tractor Structure ------------------
% Tractor One
tractor1X = 14; % lateral position (m)
tractor1Y = 5; % longitudinal position (m)

% Tractor 1
MT865_1 = initialize_MT865_structure( tractor1X, tractor1Y, 0, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor1(1:nTimeParam.nTimeStep) = MT865_1;
controlArchitecture1 = controlArchitecture;
%controlArchitecture1.structTractionController.FlagTCisOn = 0;

% Tractor 2
tractor2X = 14; % lateral position (m)
tractor2Y = 15; % longitudinal position (m)

% Tractor 2
MT865_2 = initialize_MT865_structure( tractor2X, tractor2Y, 0, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor2(1:nTimeParam.nTimeStep) = MT865_2;
controlArchitecture2 = controlArchitecture;

% Tractor 3
tractor3X = 14; % lateral position (m)
tractor3Y = 25; % longitudinal position (m)

% Tractor 3
MT865_3 = initialize_MT865_structure( tractor3X, tractor3Y, 0, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor3(1:nTimeParam.nTimeStep) = MT865_3;
controlArchitecture3 = controlArchitecture;

% Tractor 4
tractor4X = 14; % lateral position (m)
tractor4Y = 35; % longitudinal position (m)

% Tractor 4
MT865_4 = initialize_MT865_structure( tractor4X, tractor4Y, 0, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor4(1:nTimeParam.nTimeStep) = MT865_4;
controlArchitecture4 = controlArchitecture;

% Tractor 5
tractor5X = 14; % lateral position (m)
tractor5Y = 45; % longitudinal position (m)

% Tractor 5
MT865_5 = initialize_MT865_structure( tractor5X, tractor5Y, 0, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor5(1:nTimeParam.nTimeStep) = MT865_5;
controlArchitecture5 = controlArchitecture;

%% ------------------------ Simulate Tractors -----------------------------
[tractor1, controlArchitecture1, inputMat1, nTimeParam1] = tractor_simulate(tractor1, inputMat1, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture1);
[tractor2, controlArchitecture2, inputMat2, nTimeParam2] = tractor_simulate(tractor2, inputMat1, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture2);
[tractor3, controlArchitecture3, inputMat3, nTimeParam3] = tractor_simulate(tractor3, inputMat1, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture3);
[tractor4, controlArchitecture4, inputMat4, nTimeParam4] = tractor_simulate(tractor4, inputMat1, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture4);
[tractor5, controlArchitecture5, inputMat5, nTimeParam5] = tractor_simulate(tractor5, inputMat1, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture5);

%---------------------------- Plot Result -------------------------------
plot_result(tractor1,inputMat1,'r',nConstantMT865,nConstantTerrain, nTimeParam1, 1)
plot_result(tractor2,inputMat2,'b',nConstantMT865,nConstantTerrain, nTimeParam2, 0)
plot_result(tractor3,inputMat3,'g',nConstantMT865,nConstantTerrain, nTimeParam3, 0)
plot_result(tractor4,inputMat4,'c',nConstantMT865,nConstantTerrain, nTimeParam4, 0)
plot_result(tractor5,inputMat5,'m',nConstantMT865,nConstantTerrain, nTimeParam5, 0)

controlArchitecture1.structDTKF.plotSmooth = 'noPlotSmooth';
plot_DTKF_result(controlArchitecture1.structDTKF, 1, 1:2,nConstantTerrain, nConstantMT865, nTimeParam, 305);


plot_terrain_curves(controlArchitecture1.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam1, 1, 502, 530, 'r')
plot_terrain_curves(controlArchitecture2.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam2, 2, 506, 530, 'b')
plot_terrain_curves(controlArchitecture3.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam3, 3, 510, 530, 'g')
plot_terrain_curves(controlArchitecture4.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam4, 4, 514, 530, 'c')
plot_terrain_curves(controlArchitecture5.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam5, 5, 518, 530, 'm')


plot_terrain_hypothesis_2(controlArchitecture1.structRBE, controlArchitecture1.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, 'r.', 'plotHypothesis', 522)
plot_terrain_hypothesis_2(controlArchitecture2.structRBE, controlArchitecture2.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, 'b.', 'noPlotHypothesis',522)
plot_terrain_hypothesis_2(controlArchitecture3.structRBE, controlArchitecture3.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, 'g.', 'noPlotHypothesis', 522)
plot_terrain_hypothesis_2(controlArchitecture4.structRBE, controlArchitecture4.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, 'c.', 'noPlotHypothesis', 522)
plot_terrain_hypothesis_2(controlArchitecture5.structRBE, controlArchitecture5.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, 'm.', 'noPlotHypothesis', 522)


plot_bayes_estimation(tractor1, controlArchitecture1.structRBE, nConstantMT865, nTimeParam1, 'r.', 'r', 'plotHist', 605)
plot_bayes_estimation(tractor2, controlArchitecture2.structRBE, nConstantMT865, nTimeParam2, 'b.', 'b', 'plotHist', 605)
plot_bayes_estimation(tractor3, controlArchitecture3.structRBE, nConstantMT865, nTimeParam3, 'g.', 'g', 'plotHist', 605)
plot_bayes_estimation(tractor4, controlArchitecture4.structRBE, nConstantMT865, nTimeParam4, 'c.', 'c', 'plotHist', 605)
plot_bayes_estimation(tractor5, controlArchitecture5.structRBE, nConstantMT865, nTimeParam5, 'm.', 'm', 'plotHist', 605)

plot_traction_control( tractor1, nConstantMT865, controlArchitecture1.structTractionController, controlArchitecture1.structDTKF, inputMat1, nTimeParam, 1, 800)
plot_traction_control( tractor2, nConstantMT865, controlArchitecture2.structTractionController, controlArchitecture2.structDTKF, inputMat2, nTimeParam, 2, 802)
plot_traction_control( tractor3, nConstantMT865, controlArchitecture3.structTractionController, controlArchitecture3.structDTKF, inputMat3, nTimeParam, 3, 804)
plot_traction_control( tractor4, nConstantMT865, controlArchitecture4.structTractionController, controlArchitecture4.structDTKF, inputMat4, nTimeParam, 4, 806)
plot_traction_control( tractor5, nConstantMT865, controlArchitecture5.structTractionController, controlArchitecture5.structDTKF, inputMat5, nTimeParam, 5, 808)