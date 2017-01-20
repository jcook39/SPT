%% Joshua Thomas Cook 02/08/2016 Dartmouth College:
% MT865 Tractor-Sled Simulation

%% Add Paths for common functions
addpath('/Users/joshuacook/Desktop/Final_Tractor_Models_Code/Functions_All_Models')
addpath('/Users/joshuacook/Desktop/Final_Tractor_Models_Code/Functions_Multi-Body_Tractor Model_Without_Winch')

%% Clear Work Space and Close Existing Windows
clear, clc, close all

%% Initialize Structure of constant tractor parameters
resCoeff = 0.1;
bladderNo = 8;
nConstantMT865 = initialize_constant_tractors_parameters(resCoeff,bladderNo);

%% Initialize Terrain Inputs
nConstantTerrain.nominalFrictionAngle = 27;
nConstantTerrain.correlationCoefficientCohesion = 0.8;
nConstantTerrain.correlationCoefficientK = 75; % inverse correlation coefficient
nConstantTerrain.stdDev = 0.4;
nConstantTerrain.gridSizeXM = 1000;
nConstantTerrain.gridSizeYM = 600;
nConstantTerrain.gridResolutionM = 2;
nConstantTerrain.Mode = 'Constant';
%nConstantTerrain.Mode = 'TerrainVary2';
nConstantTerrain = generate_terrain(nConstantTerrain);
fprintf('The terrain map has been generated \n')

%% Set Simulation Integration Time Steps
timeStepS = 1/10; % 1/Hz
simulationTime = 60;
timeArray = [0:timeStepS:simulationTime].'; % Time array based on sample time and total simulation time
nTimeStep = size(timeArray,1); % Total number of time steps

%% Set Open Loop Control Inputs
%throttleTractorOne =        [0.45 0.6 0.7 0.8   0.9    0.9   0.9    0.9     0.9    0.9   0.9  0.9    0.9    0.9   0.9   0.9].';%
%gearTractorOne =            [4.0  4.0 5.0 6.0   6.0    6.0   6.0    6.0     6.0    7.0   8.0  9.0    9.0    9.0   9.0   9.0].'; 
% gearTractorOne =            [4.0  4.0 4.0 5.0   6.0    6.0   7.0    8.0     9.0    10.0   10.0  10.0   10.0    10.0   9.0   9.0].';
throttleTractorOne =        [0.9   0.9    0.9   0.9    0.9    0.9     0.9   0.96    0.96   0.96    0.96  0.96  0.96    0.96   0.96  0.96].';%
gearTractorOne =            [9.0   9.0   10.0  10.0   11.0    11.0   11.0   12.0    12.0   12.0   12.0   12.0  12.0    12.0   12.0  12.0].'; 
steerAngleDegTractorOne =   [0.0   0.0   0.0   0.0    0.0     0.0   -2.5    0.0     0.0    0.0    -2.5   -2.5   0.0    0.0    0.0   0.0].'./170; % + Steer angle -> more torque right track -> turns to the left
clutchCmdOne =              [0.8   0.8   0.8   0.8    1.0     1.0    1.0    1.0     1.0    1.0     1.0   1.0    1.0    1.0    1.0   0.0].';
timeScheduleInput =         [0.0   1.0   3.0   5.0    8.0     10.0   14.0   20.0    25.0   30.0    33.0  40.0   50.0   60.0  70.0  155.0].';
wayPointHeadingX =          [1000  1000  1000  1000   1000    1000   1000   1000    1000   1000    1000  1000   1000   1000  1000  1000].';
wayPointHeadingY =          [210   210   210   210    210     210    210    210     20     20       20    20     20    20   300    300].';

inputMatOne = [throttleTractorOne gearTractorOne steerAngleDegTractorOne clutchCmdOne timeScheduleInput];
inputMatOne = inputMat_make( inputMatOne, timeStepS );

inputMatTwo = inputMatOne;

%% Set WayPoint Locations for simulation
wayPointFlagMat  = [wayPointHeadingX wayPointHeadingY];
wayPointFlagMatNew  = wayPointMat_make(wayPointFlagMat,timeScheduleInput, timeStepS );

%% Initialize Controller Parameters 
Kph = 2;
Kih = 0.1;
Kpv = 2;
Kiv = 1;
kx = 0.25; % orignal value was 0.1
refDlong = -50;
refDlat = -1;
controllerStruct = initialize_velocity_heading_control(Kpv, Kiv, Kph, Kih, kx, refDlong, refDlat, timeArray);
controllerLead = controllerStruct;
controllerFollowOne = controllerStruct;

%% WayPoint Generation Parameters
wayPointTimeStepS = 1; % 1/Hz
[waypoint, nWayPointFollowEval] = initialize_waypoint_paramters(wayPointTimeStepS, timeArray, simulationTime);

%% Set Starting Location and Tractor Color
tractorOneX = 120; % lateral position (m)
tractorOneY = 210; % longitudinal position (m)
tractorOneTheta = 0; % course or yaw angle (rad)
tractorOneColor = 'r';

tractorTwoPos = [tractorOneX tractorOneY 0].' + rotation_matrix_z(tractorOneTheta)*[refDlong refDlat  0].';
tractorTwoX = tractorTwoPos(1,1); % lateral position (m)
tractorTwoY = tractorTwoPos(2,1); % longitudinal position (m)
tractorTwoTheta = tractorOneTheta; % course or yaw angle (rad)
tractorTwoColor = 'b';
tractorTwoColorWayPoint =  'bx';

%% Initialize Structure of Time Varying Tractor Torques, Forces, Inputs, States
startCondition = 'steadyState';     % vehicle is already in motion
%startCondition = 'zero';            % vehicle starts from rest

tractorOne = initialize_MT865_structure(tractorOneX,tractorOneY,tractorOneTheta,nConstantMT865,nConstantTerrain,startCondition,inputMatOne);
tractorOne(1:nTimeStep) = tractorOne;

tractorTwo = initialize_MT865_structure(tractorTwoX,tractorTwoY,tractorTwoTheta,nConstantMT865,nConstantTerrain,startCondition,inputMatTwo);
tractorTwo(1:nTimeStep) = tractorTwo;


%% %%%%%%%%%%%%%%%%% Simulation Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for timeStepNo = 2:nTimeStep
    
    [inputMatOne(timeStepNo-1,:), controllerLead] = heading_control(tractorOne(timeStepNo-1), inputMatOne(timeStepNo-1,:), controllerLead, wayPointFlagMatNew(timeStepNo-1,:).', timeStepS, timeStepNo);
    [tractorOne(timeStepNo)] = dynamics_MB_tractor(tractorOne(timeStepNo-1),inputMatOne(timeStepNo-1,:),nConstantMT865,nConstantTerrain,timeStepS);
    
    [inputMatTwo(timeStepNo-1,:), controllerFollowOne, waypoint, nWayPointFollowEval] = speed_heading_control(tractorOne(timeStepNo-1), tractorTwo(timeStepNo-1), inputMatTwo(timeStepNo-1,:), controllerFollowOne, waypoint, nWayPointFollowEval, timeStepS, timeStepNo);
    [tractorTwo(timeStepNo)] = dynamics_MB_tractor(tractorTwo(timeStepNo-1),inputMatTwo(timeStepNo-1,:),nConstantMT865,nConstantTerrain,timeStepS);

    fprintf('Simulation Time Complete: %f \n',timeStepNo*timeStepS - timeStepS)
    
    
end

%% %%%%%%%%%%%%%%%%%%%% SHOW RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_result(tractorOne,inputMatOne,'r',nTimeStep,nConstantTerrain,nConstantMT865,timeStepS,tractorOneColor,'plotContour');
plot_way_points(waypoint,nWayPointFollowEval,'kx','wo');
plot_result(tractorTwo,inputMatTwo,'b:',nTimeStep,nConstantTerrain,nConstantMT865,timeStepS,tractorTwoColor,0);
%latex_figures()


%plot_heading_control_performance(controllerLead,wayPointFlagMatNew,timeStepS,nTimeStep);
%plot_follower_speed_heading_control(controllerFollowOne,timeStepS,nTimeStep)


%  
% videoType = 'video'; % 'matlab' to play back in matlab, 'video' for windows video file
% plotTerrain = 'yes';
% nTractor.tractorOne = tractorOne;
% nTractor.tractorTwo = tractorTwo;
% nTractorColor.tractorOneColor = tractorOneColor;
% nTractorColor.tractorTwoColor = tractorTwoColor;
% simulation_video( nTractor, nTractorColor, nConstantMT865, nConstantTerrain, waypoint, timeStepS, nTimeStep ,videoType, plotTerrain, 'tractorTestJoT.avi');
