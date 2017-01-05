function MT865 = initialize_MT865_structure(X,Y,theta,nConstantMT865,nConstantTerrain,input, initialCondString)

% --------------------- Unpack Inputs -------------------------------------
gear = input(2);
valvePos = input(5);
if valvePos ~=2
   error('The winch must start in the locked position. This corresponds to valve position 2') 
end

% ------------------- Unpack Needed Tractor Paramters ---------------------
rollingRadiusM = nConstantMT865.rollingRadiusM;
nGR = nConstantMT865.nGearRatio;
GR = nGR(gear);
FD = nConstantMT865.finalDriveRatio;

% hydraulic implement parameters
GRm = nConstantMT865.gearRatioMotor;
Dm = nConstantMT865.displacementMotorM3;

% ----------------------- Initialize States -------------------------------
if strcmp('ZERO',initialCondString)
    % Body States
    vx = 0;
    vy = 0;
    r = 0;
    wl = 0;
    wr = 0;
    slipLeft = 0;
    slipRight = 0;
    bodyState = [X Y theta vx vy r wl wr].';
    
    % Power-train states
    engThrtl = 0.1;
    engSpeedRadPS = 1350*((2*pi)/60);
    clutchCom = 0;
    engCtrlThrtl = 0.1;
    powerTrainState = [engThrtl engSpeedRadPS clutchCom engCtrlThrtl].';
    
    % additional states due to hydraulic implement
    psiWinchRad = 10*pi;
    Xsled = X - nConstantMT865.tractorCG2WinchM + psiWinchRad*nConstantMT865.winchRadiusM;
    vXsled = vx;
    psiWinchRadPS = 0;
    hydPressureH = 0*6894.76; % psi to N/m^2;
    hydPressureO = 2600*6894.76;
    addState = [Xsled vXsled psiWinchRad psiWinchRadPS hydPressureH hydPressureO].';
    
    % Concatanate whole state
    state = [bodyState; powerTrainState; addState];
    
elseif strcmp('pseudo_rest',initialCondString)
    % Body States
    vx = 0.72;
    vy = 0;
    r = 0;
    wl = 1;
    wr = 1;
    slipLeft = 1 - vx/(rollingRadiusM*wl);
    slipRight = 1 - vx/(rollingRadiusM*wr);
    bodyState = [X Y theta vx vy r wl wr].';
    
    % power-train states
    engThrtl = 0.1;
    engSpeedRadPS = ((wl+wr)/2)*GR*FD;
    clutchCom = 0.7;
    engCtrlThrtl = 0.1;
    powerTrainState = [engThrtl engSpeedRadPS clutchCom engCtrlThrtl].';
    
    % additional states due to hydraulic implement
    psiWinchRad = 10*pi;
    Xsled = X - nConstantMT865.tractorCG2WinchM + psiWinchRad*nConstantMT865.winchRadiusM;
    vXsled = vx;
    psiWinchRadPS = 0;
    hydPressureH = 0*6894.76; % psi to N/m^2;
    hydPressureO = 2600*6894.76;
    addState = [Xsled vXsled psiWinchRad psiWinchRadPS hydPressureH hydPressureO].';
    
    % Concatanate whole state
    state = [bodyState; powerTrainState; addState];
    
end


% ---------------------- Initialize Structure -----------------------------

% States
MT865.state = state; 

% Other
MT865.slip = [slipLeft slipRight].';
MT865.forces = zeros(8,1);

% Transmission
MT865.transOutputSpeedRadSec = 0;
MT865.transTorqueInputNM = 0;
MT865.transTorqueOutputNM = 0;
MT865.steerMotorTorqueNM = 0;
MT865.clutchIsSlip = 1;
MT865.torqueClutchNM = 0;

% Determine if winch is Locked Based on Initial Valve Position
if valvePos == 1 % Pulling In 
    MT865.winchIsLocked = 0;
    MT865.torqWinchNM = 0;
    MT865.pSetBrakeValve = 0;
elseif valvePos == 2 % Brake Pos
    MT865.winchIsLocked = 1;
    MT865.torqWinchNM = Dm*hydPressureO*GRm;
    MT865.pSetBrakeValve = 2700*6894.76; 
elseif valvePos == 3 % Let Sled Pull Out
    MT865.winchIsLocked = 0;
    MT865.torqWinchNM = 0;
    MT865.pSetBrakeValve = 0;
end

    
% AG Hydraulics
MT865.displacementPumpM3 = 0;
MT865.engTorq2AgPumpNM = 0;
MT865.torqWinchNM = 0;
MT865.FlowH = 0;
MT865.FlowO = 0;

% Winch Controller
MT865.Error = 0;
MT865.intError = 0;
MT865.pSetDisPump = 0;

% Drawbar Force and xdot
MT865.xdot = zeros(size(state));
MT865.drawBarPullN = 0;

% Position of Track and Terrain Parameters
MT865.posLeftFront = 0;
MT865.posRightFront = 0;
MT865.posLeftRear = 0;
MT865.posRightRear = 0;
MT865.terrainLeftFront = [0 0].'; % cohesion , frictionAngle
MT865.terrainRightFront = [0 0].'; % 
MT865.terrainLeftRear = [0 0].'; % 
MT865.terrainRightRear = [0 0].'; % 

MT865 = position_nTrack( MT865, nConstantMT865 );
MT865 = terrain_parametrs_track(MT865,nConstantTerrain);

end