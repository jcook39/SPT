function [countInt, controllerFlag] = controller_counter_func(updateRateHz, countInt, timeStepS)
% This is general purpose function for determining whether the controller
% should be active at a given time step based on its update rate. This is
% so that the simulation time step "timeStepS" can run at a different rate
% that a given controller "updateRateHz"


% ------------ Increment Counter and Compute Time Since Last Control Action
updateRateSec = 1/updateRateHz;
countInt = countInt + 1;
timeSinceLastControl = timeStepS*countInt;

% ----------------- Logic for Controller Activation -----------------------
if timeSinceLastControl >= updateRateSec
    countInt = 0;
    controllerFlag = 1;
else
    controllerFlag = 0;
end



end
