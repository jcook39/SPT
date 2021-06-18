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
nConstantTerrain.nLocX(1:3,1:2) = [0 30; 30 130; 130 1000]; 
nConstantTerrain.nLocY(1:3,1:2) = [-50 50; -50 50; -50 50];
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(1:3,1) = [6.1 3 8].'; 
nConstantTerrain.frictionAngle(1:3,1) = [20 17.8 20].';
nConstantTerrain.n(1:3,1) = [1 1 1].';
nConstantTerrain.keq(1:3,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(1:3,1) = [2 0.7 2].';
nConstantTerrain.S(1:3,1) = [0.6/33 0.8/33 0.6/33].';

nConstantTerrain.terrainActual = [nConstantTerrain.cohesion.'; nConstantTerrain.frictionAngle.'; ...
    nConstantTerrain.n.'; nConstantTerrain.keq.'; nConstantTerrain.K.'; nConstantTerrain.S.'];
% Grid Specifications
nConstantTerrain.gridSizeXM = 1000;
nConstantTerrain.gridSizeYM = 50;
nConstantTerrain.gridResolutionM = 0.1;
nConstantTerrain.Mode = 'Region';
nConstantTerrain = generate_terrain(nConstantTerrain,nConstantMT865);

%% --------------- Set Simulation Integration Time Steps ------------------
nTimeParam.timeStepS = 0.05;
nTimeParam.simulationTime = 180;
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
timeScheduleInput1 =    [0    4     8   12   16   18   22   26   45  35   40  91  100  110  250].';
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
structWinchController.FlagWCisOn = 1;
controlArchitecture.structWinchController = structWinchController;


%% ------------ Initialize Tractor and Tractor Structure ------------------
% Tractor One
tractor1X = 14; % lateral position (m)
tractor1Y = 5; % longitudinal position (m)

% Tractor 1
MT865_1 = initialize_MT865_structure( tractor1X, tractor1Y, 0, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor1(1:nTimeParam.nTimeStep) = MT865_1;
controlArchitecture1 = controlArchitecture;

%% ------------------------ Simulate Tractors -----------------------------
[tractor1, controlArchitecture1, inputMat1, nTimeParam1] = tractor_simulate(tractor1, inputMat1, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture1);

% ---------------------------- Plot Result --------------------------------
Xmax = 350;
Ymax = 10 ;
plotTerrain = 1;
plot_result(tractor1,inputMat1,'r',nConstantMT865,nConstantTerrain, nTimeParam1, controlArchitecture1.structTractionController, plotTerrain, Xmax, Ymax)

controlArchitecture1.structDTKF.plotSmooth = 'noPlotSmooth';
plot_DTKF_result(controlArchitecture1.structDTKF, 1, 1:2,nConstantTerrain, nConstantMT865, nTimeParam, 305);
% plot_DTKF_result(controlArchitecture2.structDTKF, 1, 1:2,nConstantTerrain, nConstantMT865, nTimeParam, 310);
% plot_DTKF_result(controlArchitecture3.structDTKF, 1, 1:2,nConstantTerrain, nConstantMT865, nTimeParam, 315);
% plot_DTKF_result(controlArchitecture4.structDTKF, 1, 1:2,nConstantTerrain, nConstantMT865, nTimeParam, 320);
% plot_DTKF_result(controlArchitecture5.structDTKF, 1, 1:2,nConstantTerrain, nConstantMT865, nTimeParam, 325);

plot_terrain_curves(controlArchitecture1.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam1, 1, 502, 530, 'r')

plot_terrain_hypothesis_2(1,controlArchitecture1.structRBE, controlArchitecture1.structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, 'r.','r', 'plotHypothesis', 522)


plot_bayes_estimation(tractor1, controlArchitecture1.structRBE, nConstantMT865, nTimeParam1, 'r.', 'r', 'plotHist', 605)

plot_traction_control( tractor1, nConstantMT865, controlArchitecture1.structTractionController, controlArchitecture1.structDTKF, inputMat1, nTimeParam, 1, 800)


videoType = 'video'; % 'matlab' to play back in matlab, 'video' for windows video file
plotTerrain = 'no';
nTractor.tractorOne = tractor1;
nTractorColor.tractorOneColor = 'r.';
simulation_video( nTractor, nTractorColor, inputMat1, nConstantMT865, nConstantTerrain, controlArchitecture1.structTractionController,...
    nTimeParam.timeStepS, nTimeParam.nTimeStep ,videoType, plotTerrain, 'tractorThesisPresRealTime5.avi');
