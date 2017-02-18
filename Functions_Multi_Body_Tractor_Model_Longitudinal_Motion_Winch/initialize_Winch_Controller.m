function structWinchController = initialize_Winch_Controller(nTimeParam)

% ---------------------- Unpack nTimeParam --------------------------------
time = nTimeParam.time;

% ----------------------- Threshold values for activation -----------------
structWinchController.vHatLowLim = 0.5;
structWinchController.slipHatHighLim = 45;

% ----------------------- Record discrete valve position ------------------
structWinchController.discreteValvePosition = zeros(numel(time),1);

% Use winch closed loop inputs if the controller criteria for activation is met
structWinchController.FlagWCisOn = 1;

% Record if the the winch controller has been activated
structWinchController.winchControlIsOn = zeros(numel(time),1);


% Controller update rate and counters
structWinchController.winchControllerUpdateRateHz = 4;
structWinchController.winchControllerCountInt = zeros(numel(time),1);
structWinchController.winchControllerFlag = zeros(numel(time),1);

% pSet Reference Input
structWinchController.winchControllerpSetRef = zeros(numel(time),1);
structWinchController.winchControllerpSetIncrementPSIMax = 15;
structWinchController.winchControllerpSetIncrementPSIMin = 5;
structWinchController.winchControllerpSetIncrementPSI = ...
    structWinchController.winchControllerpSetIncrementPSIMax * ones(numel(time),1); 

structWinchController.attenuationFactor = 0.9;

structWinchController.winchControllerpSetPSIReduceDBFast = 200;

% Unit Conversion
structWinchController.psi2Pa = 6894.76;

end