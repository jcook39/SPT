function [ output_args ] = gear_shit_control( structTractionController, timeStepS )

% --------- Unpack Controller Structure: structTractionController ---------
gearShiftControlIsOn = structTractionController.gearShiftControlIsOn(timeStepNo-1);
gearShiftControlCountInt = structTractionController.gearShiftControlCountInt(timeStepNo-1);
gearShiftControlUpdateRateHz = structTractionController.gearShiftControlUpdateRateHz;



% ---------- Logic for Activating/Deactivating Gear Shift Controller ------
if ~gearShiftControlIsOn
    if tractionControlIsOn
       gearShiftControlIsOn = 1;
       gearShiftControlCountInt = (gearShiftControlUpdateRateHz/timeStepS) - 1;
    end
end





% -------------------- Gear Shift Controller ------------------------------
[gearShiftControlCountInt, gearShiftFlag] = controller_counter_func(gearShiftControlUpdateRateHz, gearShiftControlCountInt, timeStepS);
if gearShiftFlag
    [ gearNo ] = gear_shift_controller( structTractionController, omegaRef, omegaHat, nConstantMT865 );
    structTractionController.gearNo(timeStepNo,1) = gearNo;
    input(2) = gearNo;
else
    structTractionController.gearNo(timeStepNo,1) = structTractionController.gearNo(timeStepNo-1,1);
    gearNo = structTractionController.gearNo(timeStepNo,1);
    input(2) = gearNo;
end


end

