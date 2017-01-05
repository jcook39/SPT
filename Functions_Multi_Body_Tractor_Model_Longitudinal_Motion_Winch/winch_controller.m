function [MT865] = winch_controller(MT865,input,nConstantMT865,valvePositionIsChange)

%% Unpack Needed Constants
KiPull = nConstantMT865.KiPull;
KpPull = nConstantMT865.KpPull;
KiBrake = nConstantMT865.KiBrake;
KpBrake = nConstantMT865.KpBrake;
rs = nConstantMT865.rollingRadiusM;
rw = nConstantMT865.winchRadiusM;

%% Unpack Needed Input, State and Other
valvePosition = input(5);
pSet = input(6);

wl = MT865.state(7);
vx = MT865.state(4);
psiWinchRadPS = MT865.state(16);

winchSpdMPS = psiWinchRadPS*rw;
slipActual = 1 - (vx/(wl*rs));
intError = MT865.intError;

if valvePositionIsChange, intError = 0; end % Reset Error on Controller

%% Calculate Error and Integral Error Term
% Error = slipRef - slipActual;
%intError = intError + Error;
Error = 0; intError = 0;

% PI control for Pulling in Winch
if valvePosition == 1;
%     pSetDisPump = KiPull*intError + KpPull*Error;
%     if pSetDisPump > 1, pSetDisPump = 1; 
%     elseif pSetDisPump < -1, pSetDisPump = -1; end
    pSetDisPump = pSet;
    pSetBrakeValve = 0;
end

% PI Control for Braking
if valvePosition == 2
%     pSetBrakeValve = KiBrake*intError + KpBrake*Error;
%     if pSetBrakeValve > 2800*6874.76, pSetBrakeValve = 2800*6785.76;
%     elseif pSetBrakeValve < 0, pSetBrakeValve = 0; end
%     fprintf('pSetBrakeValve = %d \n',pSetBrakeValve/6894.76)
    pSetBrakeValve = pSet;
    pSetDisPump = 0;
end

% No Control for Letting Sled pull out Winch
if valvePosition == 3
    pSetDisPump = 0;
    pSetBrakeValve = 0;
end
    
% Pack Up Output 
MT865.Error = Error;
MT865.intError = intError;
MT865.pSetDisPump = pSetDisPump;
MT865.pSetBrakeValve = pSetBrakeValve;

end