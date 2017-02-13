function structWinchController = initialize_Winch_Controller(nTimeParam)

% ---------------------- Unpack nTimeParam --------------------------------
time = nTimeParam.time;

structWinchController.vHatLowLim = 0.5;
structWinchController.slipHatHighLim = 50;
structWinchController.discreteValvePosition = zeros(numel(time),1);
structWinchController.FlagWCisOn = 1;

structWinchController.winchControlIsOn = zeros(numel(time),1);

end