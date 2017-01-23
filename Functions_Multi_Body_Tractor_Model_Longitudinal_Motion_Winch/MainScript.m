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
nConstantTerrain.nLocX(1:3,1:2) = [0 30; 30 120; 120 150]; 
nConstantTerrain.nLocY(1:3,1:2) = [0 10; 0 10; 0 10];
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(1:3,1) = [6.1 3 6.3].'; 
nConstantTerrain.frictionAngle(1:3,1) = [20 18.1 20].';
nConstantTerrain.n(1:3,1) = [1 1 1].';
nConstantTerrain.keq(1:3,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(1:3,1) = [2 5.7 4].';
nConstantTerrain.S(1:3,1) = [0.6/33 0.8/33 0.6/33].';

% --------------------------- Tractor2 ------------------------------------
nConstantTerrain.nLocX(4:6,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(4:6,1:2) = nConstantTerrain.nLocY(1:3,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(4:6,1) = [6.1 3.5 6.3].'; 
nConstantTerrain.frictionAngle(4:6,1) = [20 17.1 20].';
nConstantTerrain.n(4:6,1) = [1 1 1].';
nConstantTerrain.keq(4:6,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(4:6,1) = [2 2.7 4].';
nConstantTerrain.S(4:6,1) = [0.6/33 0.8/33 0.6/33].';

% --------------------------- Tractor3 ------------------------------------
nConstantTerrain.nLocX(7:9,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(7:9,1:2) = nConstantTerrain.nLocY(4:6,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(7:9,1) = [6.1 4.5 6.3].'; 
nConstantTerrain.frictionAngle(7:9,1) = [20 18.1 20].';
nConstantTerrain.n(7:9,1) = [1 1 1].';
nConstantTerrain.keq(7:9,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(7:9,1) = [2 2.7 4].';
nConstantTerrain.S(7:9,1) = [0.6/33 0.8/33 0.6/33].';

% --------------------------- Tractor4 ------------------------------------
nConstantTerrain.nLocX(10:12,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(10:12,1:2) = nConstantTerrain.nLocY(7:9,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(10:12,1) = [6.1 3.5 6.3].'; 
nConstantTerrain.frictionAngle(10:12,1) = [20 17.1 20].';
nConstantTerrain.n(10:12,1) = [1 1 1].';
nConstantTerrain.keq(10:12,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(10:12,1) = [2 4.7 4].';
nConstantTerrain.S(10:12,1) = [0.6/33 0.8/33 0.6/33].';

% --------------------------- Tractor5 ------------------------------------
indexTerrain5 = 13:15;
nConstantTerrain.nLocX(indexTerrain5,1:2) = nConstantTerrain.nLocX(1:3,1:2); 
nConstantTerrain.nLocY(indexTerrain5,1:2) = nConstantTerrain.nLocY(10:12,1:2) + 10;
% Terrain Vectors for each boundry
nConstantTerrain.cohesion(indexTerrain5,1) = [6.1 3.5 6.3].'; 
nConstantTerrain.frictionAngle(indexTerrain5,1) = [20 18.1 20].';
nConstantTerrain.n(indexTerrain5,1) = [1 1 1].';
nConstantTerrain.keq(indexTerrain5,1) = [500 333 500].'; % Units in Meters (Keq) - does not seem to effect peak traction pt
nConstantTerrain.K(indexTerrain5,1) = [2 2.7 4].';
nConstantTerrain.S(indexTerrain5,1) = [0.6/33 0.8/33 0.6/33].';

nConstantTerrain.terrainActual = [nConstantTerrain.cohesion.'; nConstantTerrain.frictionAngle.'; ...
    nConstantTerrain.n.'; nConstantTerrain.keq.'; nConstantTerrain.K.'; nConstantTerrain.S.'];
% Grid Specifications
nConstantTerrain.gridSizeXM = 150;
nConstantTerrain.gridSizeYM = 50;
nConstantTerrain.gridResolutionM = 0.1;
nConstantTerrain.Mode = 'Region';
nConstantTerrain = generate_terrain(nConstantTerrain,nConstantMT865);

%% --------------- Set Simulation Integration Time Steps ------------------
timeStepS = 0.05;
simulationTime = 1;
time = [0:timeStepS:simulationTime].'; % Time array based on sample time and total simulation time
nTimeStep = size(time,1); % Total number of time steps

%% ---------------- Set Initial Position and Orientation ------------------
% Tractor One
tractor1X = 14; % lateral position (m)
tractor1Y = 5; % longitudinal position (m)
tractor1Theta = 0; % course or yaw angle (rad)


%% ------------------- Set Open Loop Control Inputs -----------------------

% Tractor One - Do nothing with winch
throttleTractor1 =      [0.23 0.25  0.25 0.33 0.4 0.4  0.5  0.5  0.9 0.9  0.8 0.9 0.4 0.4 0.4].';
%gearTractor1 =          [2    3     4    5    6   7    8    9    9   9    9   9   9   9   9  ].';
gearTractor1 =          [2    3     4    5    6   7    8    8    8   8    8   8   8   8   8  ].';
steerAngleDegTractor1 = [0    0     0    0    0   0    0    0    0   0    0   0   0   0   0  ].'; 
clutchCmd1 =            [1    1     1    1    1   1    1    1    1   1    1   1   1   1   1  ].';
valvePosition1 =        [2    2     2    2    2   2    2    2    2   2    2   2   2   2   2  ].'; % 1 - Pull In, 2 - Brake Position 3, Let Sled pull Out Winch
pSet1 = [2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700 2700].'*6894.76 ;% psi to N/m^2
timeScheduleInput1 =    [0    4     8   12   16   18   22   26   45  35   40  91  24  26  30.1].';
inputMat1 = [throttleTractor1 gearTractor1 steerAngleDegTractor1 clutchCmd1 valvePosition1 pSet1 timeScheduleInput1];
inputMat1 = inputMat_make( inputMat1, timeStepS );


%% Initialize Kalman Filter and Bayes Estimator and Traction Control

% Kalman Filter
%structDTKF_1 = intialize_DTKF([1E-3 1E-5 2E6 2E5 2E7 2E8 1E6 2E5].', nConstantMT865,nTimeStep,timeStepS);
structDTKF_1 = intialize_DTKF([1E-3 1E-5 2E6 2E5 2E7 2E8 1E6 2E5].', nConstantMT865,nTimeStep,timeStepS);

% Recursive Bayes Estimate
structRBE.terrainHypDefCohesion = 1:1:8;
structRBE.terrainHypDefFrictionAngle = 20;
structRBE.terrainHypDefn = 1;
structRBE.terrainHypDefkeq = 100:100:500;
structRBE.terrainHypDefK = 0.5:1:3.5;
structRBE.terrainHypDefS = 0.6/33;
structRBE_1 = structRBE;
structRBE_1.covariance = [5e8 5e8];
structRBE_1.normalizeString = 'noNormalize';
structRBE_1.lowProbThreshold = 1e-3;
structRBE_1 = terrain_hypothesis(structRBE_1, nConstantMT865, nTimeStep);

structRBE_2 = structRBE;
structRBE_2.covariance = [2e-3 2e-3];
structRBE_2.normalizeString = 'normalize';
structRBE_2.lowProbThreshold = 1e-3;
structRBE_2 = terrain_hypothesis(structRBE_2, nConstantMT865, nTimeStep);

structRBE_3 = structRBE;
structRBE_3.terrainHypDefK = 0.5:1:7.5;
structRBE_3.covariance = structRBE_2.covariance;
structRBE_3.normalizeString = 'normalize';
structRBE_3.lowProbThreshold = 1e-4;
structRBE_3 = terrain_hypothesis(structRBE_3, nConstantMT865,nTimeStep);

% Traction Controller
structTractionController = initialize_TractionController( 0.35, 0.15, 1E-6, nConstantMT865, nTimeStep);

%% ------------------------ Initialize Structure --------------------------
% Tractor 1
MT865_1 = initialize_MT865_structure( tractor1X, tractor1Y, tractor1Theta, nConstantMT865, nConstantTerrain, inputMat1(1,:), 'pseudo_rest');
tractor1(1:nTimeStep) = MT865_1;

%% --------------------------- Simulation Loop ----------------------------
for timeStepNo = 2:nTimeStep
    
   % Tractor One
   tractor1(timeStepNo) = motion_tractor(tractor1(timeStepNo-1),inputMat1(timeStepNo-1,:),nConstantMT865,nConstantTerrain,timeStepS);
   [tractor1(timeStepNo), valvePositionIsChange1] = detect_valve_change(tractor1(timeStepNo),inputMat1(timeStepNo,:),inputMat1(timeStepNo-1,:)); 
   tractor1(timeStepNo) = winch_controller(tractor1(timeStepNo),inputMat1(timeStepNo,:),nConstantMT865,valvePositionIsChange1);
   
   %structDTKF_1 = propogate_DTKF(structDTKF_One, tractor1(timeStepNo), nConstantMT865, inputMat1(timeStepNo-1,:).', timeStepNo, 'Konline');
   structDTKF_1 = propogate_DTKF(structDTKF_1, tractor1(timeStepNo), nConstantMT865, inputMat1(timeStepNo-1,:).', timeStepNo, 'Koffline');
   
   structRBE_3 = bayes_estimation(structRBE_3, structDTKF_1, nConstantMT865, time, timeStepNo);
   
   [ structTractionController, inputMat1(timeStepNo,:) ] = traction_control( structTractionController, structRBE_3, structDTKF_1, tractor1(timeStepNo), inputMat1(timeStepNo,:), nConstantMT865, timeStepNo, timeStepS );
 
    fprintf('Simulation Time: %f seconds \n',(timeStepNo-1)*timeStepS)       
end

%% ---------------------------- Plot Result -------------------------------
plot_result(tractor1,inputMat1,'k',nConstantMT865,nConstantTerrain,nTimeStep,timeStepS,1)

plot_DTKF_result(structDTKF_1, nConstantTerrain, nConstantMT865, nTimeStep, timeStepS, 305);

plot_terrain_hypothesis_2(structRBE_3, structDTKF_1, nConstantMT865, nConstantTerrain, time, 'Actual', 502)
plot_terrain_hypothesis_2(structRBE_3, structDTKF_1, nConstantMT865, nConstantTerrain, time, 'Hypotheses', 520)

plot_bayes_estimation(tractor1, structRBE_3, nConstantMT865, time, 605)

plot_traction_control( tractor1, nConstantMT865, structTractionController, structDTKF_1, inputMat1, nTimeStep, timeStepS, 800 )