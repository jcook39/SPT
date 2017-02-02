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
structTractionController.gearShiftControlEngineRPMTargetRPM = 1700;
structTractionController.gearNo = zeros(nTimeStep,1);

structTractionController.gearShiftFlag = zeros(nTimeStep,1);
structTractionController.sysControllerD = sysControllerD;

% --------------- Traction Control is On ----------------------------------
structTractionController.FlagTCisOn = 1;

end