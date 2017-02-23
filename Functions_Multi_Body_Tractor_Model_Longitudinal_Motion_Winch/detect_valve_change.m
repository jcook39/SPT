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
       fprintf('VALVE POSITION CHANGE DETECTED \n')
       fprintf('WINCH IS UNLOCKED !!!! \n')
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