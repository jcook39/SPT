function  nConstantMT865  = initialize_constant_tractors_parameters(resCoeff,bladderNo)

% This function declares constant parameters for an MT865 tractor to be
% used in simulations in a struct called nConstant

% Units: 
%   kg = KG, 
%   meters = M 
%   per second = PS
%   per second^2 = PS2
%   newton = N

% Body Parameters:
nConstantMT865.gMPS2 = 9.81;
nConstantMT865.massTractorKG = 25000;
nConstantMT865.weightTractorN = nConstantMT865.gMPS2*nConstantMT865.massTractorKG;
nConstantMT865.normalForceTrackN = nConstantMT865.weightTractorN/2;
nConstantMT865.rollingRadiusM = 0.7851;
nConstantMT865.trackInertiaKGM2 = 300; 
nConstantMT865.trackDamp = 0;
nConstantMT865.trackWidthM = 0.9144;
nConstantMT865.trackLengthM = 3.250;
nConstantMT865.trackAreaM2 = nConstantMT865.trackLengthM*nConstantMT865.trackWidthM;
nConstantMT865.trackCG2trackCG = 2.985;
nConstantMT865.tractorIntertiaKGM2 = 150000;
nConstantMT865.tractorCG2WinchM = 2.75;

% Engine Data/Parameters
nConstantMT865.engineTorqueDataNM = [2530 2530 2512 2468 2364 2244 2128 1960 1781 1547 1335];
nConstantMT865.engineSpeedDataRadPS = [1300:100:2300].*((2*pi)/60);
nConstantMT865.engineSpeedMaxRadPS = 2100*((2*pi)/60);
nConstantMT865.engineSpeedMinRadPS = 1300*((2*pi)/60);
nConstantMT865.timeConstantClutch = 0.1;
nConstantMT865.inertiaEngineKGM2 = 1;
nConstantMT865.engineDamp = 1;

% Engine Idler controller Settings
nConstantMT865.idleSpeedReferenceRadPS = 1250*((2*pi)/60);
nConstantMT865.speedThresholdRadPS = 40*((2*pi)/60);
nConstantMT865.timeConstantIdleController = 0.5;

% Transmission Parameters
nConstantMT865.nGearNo = 1:16;
nConstantMT865.nGearRatio = [8.44205 6.64811 5.25283 4.13660 3.47409 3.08571 2.73585 ...
    2.43000 2.16166 1.92000 1.70231 1.51200 1.26984 1.00000 0.79012 0.62222 0];
nConstantMT865.finalDriveRatio = 27.35316;
nConstantMT865.transmissionTimeConstantS = 0.01;
nConstantMT865.inertiaTransmissionKGM2 = 300;
nConstantMT865.transmissionDamp = 10;
nConstantMT865.timeConstantTransmission = 0.01;

% Friction Clutch Parameters
nConstantMT865.clutchStaticFrictionCapNM = 2200;
nConstantMT865.clutchDynamicFrictionCapNM = 1800;

% Winch Parameters
rw = 0.1905;
nConstantMT865.areaBrake = pi*(0.422402^2) - pi*((0.422402 - 0.320802)^2);
nConstantMT865.brakeFrictionCoeff = 0.2;
nConstantMT865.winchRadiusM = rw;
nConstantMT865.winchInertiaKGM2 = (pi*(rw/2)^2)*0.7874*8050*(1/2)*((rw/2)^2);
nConstantMT865.winchDamp = 3;
nConstantMT865.winchCableMaxM = 50;

% Initialize Winch Controller Gains
nConstantMT865.KiPull = 0.05;
nConstantMT865.KpPull = 0.2;
nConstantMT865.KiBrake = 1E5;
nConstantMT865.KpBrake = 1E10;

% AG Hydraulic Parameters
nConstantMT865.bulkModulusNM2 = (4E5)*6984.76; % psi to N/m^2
nConstantMT865.volumeHighM3 = 0.265/40; % m^3
nConstantMT865.volumeLowM3 = 0.265/150; % 0.265/100
nConstantMT865.pressureHighMaxNM2 = 2900*6894.76; % psi to N/m^2
nConstantMT865.pressureHighSetNM2 = 2800*6894.76; % psi to N/m^2

nConstantMT865.displacementPumpM3 = 85*(1E-6); % cc/rad or cm^3/rad to m^3/rad
nConstantMT865.gearRatioPump = 1;
nConstantMT865.pumpSpeedMaxRadPS = nConstantMT865.engineSpeedMaxRadPS/nConstantMT865.gearRatioPump;
nConstantMT865.pumpMaxFlowM3S = nConstantMT865.displacementPumpM3*nConstantMT865.pumpSpeedMaxRadPS;

nConstantMT865.displacementMotorM3 = 107*(1E-6); %0.003286;
nConstantMT865.gearRatioMotor = 18;
%nConstantMT865.maxFlowBrakeValveM3 = nConstantMT865.displacementMotorM3*120; % 70 radPS
nConstantMT865.maxFlowBrakeValveM3 = 5*nConstantMT865.pumpMaxFlowM3S;

nConstantMT865.leakFlowM3S = 0.1*(1E-7); % cm^3/s to m^3/s



% Resistance Sled
massBladderKG = 10000;
nConstantMT865.resCoeff = resCoeff;
nConstantMT865.bladderNo = bladderNo;
nConstantMT865.massBladderKG = massBladderKG;
nConstantMT865.RsledN = resCoeff*bladderNo*massBladderKG*9.81;
nConstantMT865.massSledKG = massBladderKG*bladderNo;

% Sled Parameters
nConstantMT865.sledLengthM = 21;
nConstantMT865.sledWidthM = 12.5;
nConstantMT865.indSledWidthM = 2.2;
nConstantMT865.CRLTool = 6;
nConstantMT865.inertiaSledKGM2 = (1/12)*nConstantMT865.massSledKG*( (nConstantMT865.sledLengthM^2 + nConstantMT865.sledWidthM^2) ); %
nConstantMT865.TTCG2Hitch = nConstantMT865.trackLengthM/4;

% Towing Arm
nConstantMT865.towArmM = 1;


end

