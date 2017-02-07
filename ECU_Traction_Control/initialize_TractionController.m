function structTractionController = initialize_TractionController(KpAll, KiAll, KdAll, Tf, nConstantMT865, nTimeStep)

% --------------- Unpack Tractor Needed Parameters ------------------------
nGR = nConstantMT865.nGearRatio;

% ----------------- Calculate Gear ratio specific gains -------------------
KpGR = KpAll./nGR;
KiGR = KiAll./nGR;
KdGR = KdAll./nGR;

%KpGR(:) = KpAll;
%KiGR(:) = KiAll;

% ------------------- Controller Initilization ----------------------------
% PIDF

ControllerC = pid(KpGR(1:16).', KiGR(1:16).', KdGR(1:16).', Tf);
controllerSampleTime = 0.05; % 20 Hz;
sysControllerC = tf(ControllerC);
sysControllerD = c2d(sysControllerC,controllerSampleTime,'tustin');

% ----------------- Package controller structure --------------------------
structTractionController.KpGR = KpGR;
structTractionController.KiGR = KiGR;

structTractionController.errorOmegaIntegrated = zeros(nTimeStep,1);
structTractionController.errorOmega = zeros(nTimeStep,1);
structTractionController.throttleFeedForward = zeros(nTimeStep,1);
structTractionController.torqueFeedForwardNM = zeros(nTimeStep,1);
structTractionController.peakSlip = zeros(nTimeStep,1);
structTractionController.iref = zeros(nTimeStep,1);
structTractionController.omegaRef = zeros(nTimeStep,1); 
structTractionController.throttleControllerPIDFF = zeros(nTimeStep,1);
structTractionController.throttleControllerPID = zeros(nTimeStep,1);

structTractionController.tractionControlIsOn = zeros(nTimeStep,1);

structTractionController.gearShiftControlUpdateRateHz = 0.5;
structTractionController.gearShiftControlIsOn = zeros(nTimeStep,1);
structTractionController.gearShiftControlCountInt = zeros(nTimeStep,1);
structTractionController.gearNo = zeros(nTimeStep,1);

structTractionController.gearShiftFlag = zeros(nTimeStep,1);
structTractionController.sysControllerD = sysControllerD;

% --------------- Traction Control is On ----------------------------------
structTractionController.FlagTCisOn = 1;

% ---------------- Traction Control Last Throttle Input -------------------
structTractionController.lastThrottleInputTC = 0;

% ----------- Traciton Control On at Least once flag ----------------------
structTractionController.TChasBeenTurnedOnAtLeastOnce = 0;

% --------------- Store Engine Speed RadPS --------------------------------
structTractionController.engSpeedRadPS = zeros(nTimeStep,1);
structTractionController.engSpeedRadPS(1,1) = 1800*((2*pi)/60);
% Put first value within bounds so that it doesnt throw controller off

% ------------------- Max Power RPM Gear Controller Parameters ------------
structTractionController.gearShiftControlEngineRPMTargetRPM = 1700;

% ------------------ RPM Bounds Gear Controller Paramters -----------------
structTractionController.gearShiftControlLowEngineRPMBound = 1350;
structTractionController.gearShiftControlHighEngineRPMBound = 1950;

% --- Set minimum sprocket speed in any gear ratio based on engine RPM ----
structTractionController.minimumEngineSpeedRPMRef = ...
    structTractionController.gearShiftControlLowEngineRPMBound - 50;


end