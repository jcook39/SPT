function structTractionController = initialize_TractionController(KpAll, KiAll, nConstantMT865, nTimeStep)


% --------------- Unpack Tractor Needed Parameters ------------------------
nGR = nConstantMT865.nGearRatio;

% ----------------- Calculate Gear ratio specific gains -------------------
KpGR = KpAll./nGR;
KiGR = KiAll./nGR;

KpGR(:) = KpAll;
KiGR(:) = KiAll;

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
structTractionController.throttleController = zeros(nTimeStep,1);

structTractionController.tractionControlIsOn = zeros(nTimeStep,1);

structTractionController.gearShiftControlUpdateRateHz = 1;
structTractionController.gearShiftControlIsOn = zeros(nTimeStep,1);
structTractionController.gearShiftControlCountInt = zeros(nTimeStep,1);
structTractionController.gearShiftControlEngineRPMTargetRPM = 1800;
structTractionController.gearNo = zeros(nTimeStep,1);

structTractionController.gearShiftFlag = zeros(nTimeStep,1);

end