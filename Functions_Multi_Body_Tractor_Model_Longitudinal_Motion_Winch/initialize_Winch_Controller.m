function structWinchController = initialize_Winch_Controller(nTimeParam)

% ---------------------- Unpack nTimeParam --------------------------------
time = nTimeParam;

structWinchController.vHatLowLim = 0.5;
structWinchController.discreteValvePosition = zeros(numel(time),1);

structWinchController.FlagWCisOn = 1;

end