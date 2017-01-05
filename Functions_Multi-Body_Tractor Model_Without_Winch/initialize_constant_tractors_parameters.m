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
nConstantMT865.trackDamp = 10;
nConstantMT865.trackWidthM = 0.9144;
nConstantMT865.trackLengthM = 3.250;
nConstantMT865.trackAreaM2 = nConstantMT865.trackLengthM*nConstantMT865.trackWidthM;
nConstantMT865.trackCG2trackCG = 2.985;
nConstantMT865.tractorIntertiaKGM2 = 150000;

% Engine Data/Parameters
nConstantMT865.engineTorqueDataNM = [2530 2530 2512 2468 2364 2244 2128 1960 1781 1547 1335];
nConstantMT865.engineSpeedDataRadPS = [1300:100:2300].*((2*pi)/60);
nConstantMT865.timeConstantClutch = 0.1;
nConstantMT865.inertiaEngineKGM2 = 1;
nConstantMT865.engineDamp = 1;

% Transmission Parameters
nConstantMT865.nGearRatio = [8.44205 6.64811 5.25283 4.13660 3.47409 3.08571 2.73585 ...
    2.43000 2.16166 1.92000 1.70231 1.51200 1.26984 1.00000 0.79012 0.62222 0];
kilometersPerHour2MetersPerSecond = 0.277778;
nConstantMT865.maxSpeednGearRatio = [2.67 3.39 4.29 5.45 6.49 7.30 8.24 9.28 10.43...
    11.74 13.24 14.91 17.75 22.54 28.53 36.22 39.67].'*kilometersPerHour2MetersPerSecond;

nConstantMT865.finalDriveRatio = 27.35316;
nConstantMT865.steerGearRatio = 100;
nConstantMT865.transmissionTimeConstantS = 0.01;
nConstantMT865.inertiaTransmissionKGM2 = 300;
nConstantMT865.transmissionDamp = 10;
nConstantMT865.timeConstantTransmission = 0.01;

% Friction Clutch Parameters
nConstantMT865.clutchStaticFrictionCapNM = 2200;
nConstantMT865.clutchDynamicFrictionCapNM = 1800;

% Towing Arm
nConstantMT865.towArmM = 1;

% Sled
massBladderKG = 10000;
nConstantMT865.mSledKG = bladderNo*massBladderKG;
nConstantMT865.RsledN = resCoeff*bladderNo*massBladderKG*9.81;
nConstantMT865.sledLengthM = 21;
nConstantMT865.sledWidthM = 12.5;
nConstantMT865.indSledWidthM = 2.2;
nConstantMT865.CRLTool = 6;
nConstantMT865.inertiaSledKGM2 = (1/12)*nConstantMT865.mSledKG*( (nConstantMT865.sledLengthM^2 + nConstantMT865.sledWidthM^2) ); %
nConstantMT865.TTCG2Hitch = nConstantMT865.trackLengthM/4;

% Mass Matrix & Force Vector
% [massMatrix, forceVector, ~ , ~] = EOMDerivTractorTrailerMultiBody1FrameBF();
% nConstantMT865.massMatrix = massMatrix;
% nConstantMT865.forceVector = forceVector;

% Differential Steering Parameters
nConstantMT865.steerAngleTau = 0.63;
nConstantMT865.volumehM3 = 0.006625;
nConstantMT865.DpMaxM3 = 120E-6;
nConstantMT865.DmM3 = 105E-6;
nConstantMT865.bulkModulus = 2.75E9;
nConstantMT865.GRpump = 1;
nConstantMT865.GRStrMotor = 160;
nConstantMT865.maxDiffTrackSpeed = 2.78; % maximum track speed difference in m/s

end

