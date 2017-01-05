%% Joshua Thomas Cook 02/08/2016 Dartmouth College:
% MT865 Tractor-Sled Simulation

%% Add Paths for common functions
addpath('Q:/NISTcontrols/Josh_Cook/Final_Tractor_Models_Code/Functions_All_Models')
addpath('Q:/NISTcontrols/Josh_Cook/Final_Tractor_Models_Code/Functions_Multi-Body_Tractor Model_Without_Winch')


%% Clear Work Space and Close Existing Windows
clear, clc, close all

%% Initialize Structure of constant tractor parameters
resCoeff = 0.13;
bladderNo = 8;
nConstantMT865 = initialize_constant_tractors_parameters(resCoeff,bladderNo);

%% Initialize Terrain Inputs
nConstantTerrain.nominalFrictionAngle = 27;
nConstantTerrain.correlationCoefficientCohesion = 0.8;
nConstantTerrain.correlationCoefficientK = 75; % inverse correlation coefficient
nConstantTerrain.stdDev = 0.4;
nConstantTerrain.gridSizeXM = 1000;
nConstantTerrain.gridSizeYM = 400;
nConstantTerrain.gridResolutionM = 2;
nConstantTerrain.Mode = 'Constant';
%nConstantTerrain.Mode = 'TerrainVary2';
nConstantTerrain = generate_terrain(nConstantTerrain);
fprintf('The terrain map has been generated \n')

%% Set Simulation Integration Time Steps
timeStepS = 0.01;
simulationTime = 20;
time = [0:timeStepS:simulationTime].'; % Time array based on sample time and total simulation time
nTimeStep = size(time,1); % Total number of time steps

%% Set Open Loop Control Inputs
throttleTractorOne =        [0.45 0.6 0.7 0.8 0.9   0.9  0.9   0.9   0.9  0.9   0.9  0.9    0.9   0.9   0.9   0.9].';
gearTractorOne =            [4.0  4.0 5.0 6.0 6.0   6.0  6.0   6.0   7.0  8.0   9.0  9.0    9.0   9.0   9.0   9.0].'; 
steerAngleDegTractorOne =   [0.0  0.0 0.0 0.0 0.0  -10.0 0.0  -10.0  0.0  0.0   0.0  0.0    10.0  5.0   0.0   0.0].'; % + Steer angle -> more torque right track -> turns to the left
clutchCmdOne =              [0.8  0.8 0.8 0.8 1.0   1.0  1.0   1.0   1.0  1.0   1.0  1.0    1.0   1.0   1.0   0.0].';
timeScheduleInput =         [0.0  1.0 3.0 5.0 8.0   10.0 14.0  15.0  18.0 20.0  23.0 25.0   28.0  35.0  40.0  155.0].';
inputMatOne = [throttleTractorOne gearTractorOne steerAngleDegTractorOne clutchCmdOne timeScheduleInput];
inputMatOne = inputMat_make( inputMatOne, timeStepS );

inputMatTwo = inputMatOne;

fprintf('The input matrix has been generated \n')

%% Initialize Controller Parameters
controller.Kpx = 0.3;
controller.Kix = 0.02;
% controller.Kpy = 0.5;      % y distance controller paramterization
% controller.Kiy = 0.0001;
controller.Kpy = 2;
controller.Kiy = 0.1;
controller.Refx1 = -20;
controller.Refy1 = -20;
controller.error = zeros(3,1);
controller.integratedError = zeros(3,1);
controller(1:nTimeStep) = controller;

%% Set Starting Location and Tractor Color
tractorOneX = 60; % lateral position (m)
tractorOneY = 210; % longitudinal position (m)
tractorOneTheta = 0; % course or yaw angle (rad)

tractorTwoX = tractorOneX - 20; % lateral position (m)
tractorTwoY = tractorOneY - 20; % longitudinal position (m)
tractorTwoTheta = 0; % course or yaw angle (rad)

%% Initialize Structure of Time Varying Tractor Torques, Forces, Inputs, States
startCondition = 'steadyState';     % vehicle is already in motion
%startCondition = 'zero';            % vehicle starts from rest

tractorOne = initialize_MT865_structure(tractorOneX,tractorOneY,tractorOneTheta,nConstantMT865,nConstantTerrain,startCondition,inputMatOne);
tractorOne(1:nTimeStep) = tractorOne;

tractorTwo = initialize_MT865_structure(tractorTwoX,tractorTwoY,tractorTwoTheta,nConstantMT865,nConstantTerrain,startCondition,inputMatOne);
tractorTwo(1:nTimeStep) = tractorTwo;

%% %%%%%%%%%%%%%%%%% Simulation Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for timeStepNo = 2:nTimeStep
    
   [tractorOne(timeStepNo)] = dynamics_MB_tractor(tractorOne(timeStepNo-1),inputMatOne(timeStepNo-1,:),nConstantMT865,nConstantTerrain,timeStepS);
   [tractorTwo(timeStepNo)] = dynamics_MB_tractor(tractorTwo(timeStepNo-1),inputMatTwo(timeStepNo-1,:),nConstantMT865,nConstantTerrain,timeStepS);
   % Control function here
   [inputMatTwo(timeStepNo,:), controller(timeStepNo)] = control_update(tractorOne(timeStepNo), tractorTwo(timeStepNo), inputMatOne(timeStepNo,:), controller(timeStepNo-1), timeStepS);
   
%    fprintf(' throttle = %f , gear = %f , steerAngle = %f \n' , inputMatOne(timeStepNo,1) , inputMatOne(timeStepNo,2), inputMatOne(timeStepNo,3))
%    fprintf(' throttle = %f , gear = %f , steerAngle = %f \n' , inputMatTwo(timeStepNo,1) , inputMatTwo(timeStepNo,2), inputMatTwo(timeStepNo,3))
%    
  %('Simulation Time Complete: %f \n',timeStepNo*timeStepS - timeStepS)
end

%% %%%%%%%%%%%%%%%%%%%% SHOW RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tractorOneColor = 'r';
tractorTwoColor = 'b';

plot_result(tractorOne,inputMatOne,'r',nTimeStep,nConstantTerrain,nConstantMT865,timeStepS,tractorOneColor,'plotContour',controller)
plot_result(tractorTwo,inputMatTwo,'b',nTimeStep,nConstantTerrain,nConstantMT865,timeStepS,tractorTwoColor,0,controller)

 
% videoType = 'video'; % 'matlab' to play back in matlab, 'video' for windows video file
% plotTerrain = 'yes';
% nTractor.tractorOne = tractorOne;
% nTractor.tractorTwo = tractorTwo;
% nTractorColor.tractorOneColor = tractorOneColor;
% nTractorColor.tractorTwoColor = tractorTwoColor;
% simulation_video( nTractor, nTractorColor, nConstantMT865, nConstantTerrain, timeStepS, nTimeStep ,videoType, plotTerrain);

