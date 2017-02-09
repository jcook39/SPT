function [MT865,input] = winch_controller(MT865,input,inputMinus1,nConstantMT865,structDTKF,structTractionController,structWinchController,timeStepNo)

% ------------------------ Unpack Needed Constants ------------------------
rs = nConstantMT865.rollingRadiusM;
rw = nConstantMT865.winchRadiusM;

% ----------------- Unpack Needed Input, State and Other ------------------
valvePosition = input(5);
pSet = input(6);

wl = MT865.state(7);
vx = MT865.state(4);
psiWinchRadPS = MT865.state(16);

% ---------------------  Unpack DTKF Parameters ---------------------------
xHatPlus = structDTKF.xHatPlus(:,timeStepNo);
vHat = xHatPlus(1);
smoothedvHat = structDTKF.smoothedvHat(1,timeStepNo); 

% ------------------- Unpack structWinchController ------------------------
vHatLowLim = structWinchController.vHatLowLim;

% -------------------- Winch Decision/Control -----------------------------
tractorBelowSpeedThreshold = (vHatLowLim >= smoothedvHat); 
% if tractorBelowSpeedThreshold
%     valvePosition = 3;
% end
input(1,5) = valvePosition;
discreteValvePosition = valvePosition;

% ----------- Detect if there's change in discrete valve position ---------
[MT865] = detect_valve_change(MT865,input,inputMinus1);

% ------------------------ Winch Commands ---------------------------------
% Pulling in Winch
if valvePosition == 1
    pSetDisPump = pSet;
    pSetBrakeValve = 0;
end

% Braking Winch
if valvePosition == 2
    pSetBrakeValve = pSet;
    pSetDisPump = 0;
end

% Let Sled Out
if valvePosition == 3
    pSetDisPump = 0;
    pSetBrakeValve = 0;
end
    
% Pack Up Output 
MT865.pSetDisPump = pSetDisPump;
MT865.pSetBrakeValve = pSetBrakeValve;

% --------------------- Pack up Winch controller structure ----------------
structWinchController.discreteValvePosition(timeStepNo,1) = discreteValvePosition;

end

function [MT865] = detect_valve_change(MT865,input,inputMinus1)

% --------------------- Unpack Inputs and States --------------------------
valvePos = input(5);
valvePosMinus1 = inputMinus1(5);
hydPrH = MT865.state(17);
hydPrO = MT865.state(18);
winchIsLocked = MT865.winchIsLocked;


% ---------------- Detect Change in Valve Input ---------------------------
valvePositionIsChange = ~(valvePos == valvePosMinus1);
    if valvePositionIsChange
       winchIsLocked = 0;
       fprintf('valve position change detected')
       if valvePos == 1
           hydPrO = 0;
       elseif valvePos == 2
           hydPrH = 0;
       elseif valvePos == 3
           hydPrH = 0;
           hydPrO = 0;
       end
    end
    
    
    
% ----------------------- Pack Up -----------------------------------------
MT865.winchIsLocked = winchIsLocked;
MT865.state(17) = hydPrH;
MT865.state(18) = hydPrO;
    
end