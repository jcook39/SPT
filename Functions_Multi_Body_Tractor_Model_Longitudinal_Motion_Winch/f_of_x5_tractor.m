function [output] = f_of_x5_tractor(x, MT865,input,nConstantMT865,outputCond)
% Propagates states for EOM for MT865 Tractor Sled System

%% Unwrap Necessary Tractor Body and Power-Train Parameters: (T-tractor, s - sprocket)
bT = nConstantMT865.trackCG2trackCG;
LT = nConstantMT865.trackLengthM;
mT = nConstantMT865.massTractorKG;
JT = nConstantMT865.tractorIntertiaKGM2;
Js = nConstantMT865.trackInertiaKGM2;
rs = nConstantMT865.rollingRadiusM;
trackDamp = nConstantMT865.trackDamp;
nGR = nConstantMT865.nGearRatio;
FD = nConstantMT865.finalDriveRatio;

transTau = nConstantMT865.timeConstantTransmission;
clutchTau = nConstantMT865.timeConstantClutch;
transDamp = nConstantMT865.transmissionDamp;
TfMaxs = nConstantMT865.clutchStaticFrictionCapNM;
TfMaxd = nConstantMT865.clutchDynamicFrictionCapNM;
Jtrans = nConstantMT865.inertiaTransmissionKGM2;
Jeng = nConstantMT865.inertiaEngineKGM2;
engDamp = nConstantMT865.engineDamp;

rw = nConstantMT865.winchRadiusM;
Jw = nConstantMT865.winchInertiaKGM2;
winchDamp = nConstantMT865.winchDamp;
winchCableMaxM = nConstantMT865.winchCableMaxM;

DpMax = nConstantMT865.displacementPumpM3;
Dm = nConstantMT865.displacementMotorM3;
B = nConstantMT865.bulkModulusNM2;
VH = nConstantMT865.volumeHighM3;
VO = nConstantMT865.volumeLowM3;
pMax = nConstantMT865.pressureHighMaxNM2;
pSetValveH = nConstantMT865.pressureHighSetNM2;
pressureMaxPumpNM2 = nConstantMT865.pressureMaxPumpNM2;
qleak = nConstantMT865.leakFlowM3S;
GRp = nConstantMT865.gearRatioPump;
GRm = nConstantMT865.gearRatioMotor;
qPumpMax = nConstantMT865.pumpMaxFlowM3S;
areaBrake = nConstantMT865.areaBrake;
brakeFricCoeff = nConstantMT865.brakeFrictionCoeff;
maxFlowBrakeValveM3 = nConstantMT865.maxFlowBrakeValveM3;

RsledN = nConstantMT865.RsledN;
mS = nConstantMT865.massSledKG;

idleSpeedReferenceRadPS = nConstantMT865.idleSpeedReferenceRadPS;
speedThresholdRadPS = nConstantMT865.speedThresholdRadPS;
timeConstantIdleController = nConstantMT865.timeConstantIdleController;

%% Unwrap Inputs and State

% Inputs
commandThrottle = input(1);
gear = input(2);
steerAngleDeg = input(3);
clutchCmd = input(4);
valvePos = input(5);
pset = input(6);
GR = nGR(gear);

pSetDisPump = MT865.pSetDisPump;
pSetBrakeValve = MT865.pSetBrakeValve;

% Body State
X = x(1);
Y = x(2);
theta = x(3);
vx = x(4)*(x(4) > 0); % Logic Binds state to be equal or greater than 0
vy = x(5);
thetaDot = x(6);
wl = x(7)*(x(7) > 0); % Logic Binds state to be equal or greater than 0
wr = x(8)*(x(8) > 0); % Logic Binds state to be equal or greater than 0


% PowerTrain State
engThrtl = x(9); % Transmission Output Toruqe
engSpdRadPS = x(10); % Engine Speed State
com = x(11);   % Clutch Engage Cmd
engCtrlThrtl = x(12); 

% Added State
Xsled = x(13);
vXsled = x(14);
psiWinchRad = x(15);
psiWinchRadPS = x(16);
hydPrH = x(17);
hydPrO = x(18);

% Unwrap Current State (In the Sense of a State Machine)
clutchIsSlip = MT865.clutchIsSlip;
winchIsLocked = MT865.winchIsLocked;
winchCableIsAtMaxLengthM = (winchCableMaxM <= rw*psiWinchRad);
if ~winchCableIsAtMaxLengthM
    RsledN = RsledN*(vXsled > 0);
end
    

%% AG Element Hydraulics

if valvePos == 1
    % Displacement of Pump
    Kpump = DpMax/(pressureMaxPumpNM2 - pSetDisPump);
    Dp = DpMax*(hydPrH <= pSetDisPump) + (DpMax - Kpump*(hydPrH-pSetDisPump))*(pSetDisPump < hydPrH)*(hydPrH < pressureMaxPumpNM2); % + 0*(hydPr >= pH);
   % Load torque and Torque Out
    %Dp = DpMax*commandDisplacement;
    engTorq2AgPumpNM = Dp*hydPrH*(1/GRp);
    torqWinchNM = Dm*hydPrH*GRm; 
    % Flow into High Pressure Section of Hydraulic System
    qValveHMax = qPumpMax;
    pumpSpeedRadPS = engSpdRadPS/GRp;
    motorSpeedRadPS = psiWinchRadPS*GRm;
    KvalveH = (qValveHMax - qleak)/(pMax - pSetValveH);
    qValveH = qleak*(hydPrH <= pSetValveH) + (qleak + KvalveH*(hydPrH - pSetValveH))*(pSetValveH < hydPrH)*(hydPrH < pMax) + qValveHMax*(hydPrH >= pMax);
    Qh = Dp*pumpSpeedRadPS + Dm*motorSpeedRadPS - qValveH;
    Qo = 0;
elseif valvePos == 2
    Dp = 0;
    engTorq2AgPumpNM = Dp*hydPrH*(1/GRp);
    torqWinchNM = Dm*hydPrO*GRm;
    motorSpeedRadPS = psiWinchRadPS*GRm;
    KvalveO = (maxFlowBrakeValveM3 - qleak)/(pMax - pSetBrakeValve);
    qValveO = qleak*(hydPrO <= pSetBrakeValve) + (qleak + KvalveO*(hydPrO - pSetBrakeValve))*(pSetBrakeValve < hydPrO)*(hydPrO < pMax) + maxFlowBrakeValveM3*(hydPrO >= pMax);
    Qh = 0;
    Qo = Dm*motorSpeedRadPS - qValveO;
elseif valvePos == 3
    Dp = 0;
    engTorq2AgPumpNM = Dp*hydPrH*(1/GRp);
    torqWinchNM = 0;
    Qh = 0;
    Qo = 0;
end

%% Attempt at Control of Throttle for max speed
    commandThrottle = commandThrottle*tanh( abs(engSpdRadPS - (2100*((2*pi)/60))) / 10); 
    
%% Calculate Torque out of the engine and how it is divided between the transmission and steering motor and Winch
    engTorqNM = engine_interp( engThrtl, engCtrlThrtl, engSpdRadPS, nConstantMT865 );
    engTorq2StrMotorNM = (steerAngleDeg/170)*engTorqNM;
    engTorq2TransNM = engTorqNM - engTorq2StrMotorNM - engTorq2AgPumpNM; 
    
%% Calculate Forces at the Track
[Fl,Fr,RL,RR,RlLF,RlRF,RlLR,RlRR,slipLeft,slipRight,sinkageLeftDynamic,sinkageRightDynamic] = track_forces(x, MT865, nConstantMT865);
%fprintf('slipLeft: %d , slipRight: %d \n',slipLeft,slipRight)

%% Wet Friction Clutch Dynamics and Logic
Tfs = com*TfMaxs;
Tfd = com*TfMaxd;
    
loadTorqueNM = (Fl + Fr)/(GR*FD);
transOutSpdRadPS = 0.5*FD*abs(wl+wr);
transInSpdRadPS = transOutSpdRadPS*GR;

% Dynamics for each Clutch State
if clutchIsSlip
   relShaftSpdRadPS = engSpdRadPS - transInSpdRadPS;
   torqClutchNM = Tfd*tanh(2*(relShaftSpdRadPS/2));
   engSpdRadPSDot = (engTorq2TransNM - engDamp*engSpdRadPS - torqClutchNM)/Jeng;
else
    shaftSpdRadPS = transInSpdRadPS;
    torqClutchNM = (Jtrans*engTorq2TransNM + Jeng*loadTorqueNM - (Jtrans*engDamp - Jeng*transDamp)*shaftSpdRadPS )/(Jtrans + Jeng);
    engSpdRadPSDot = (1/transTau)*(-engSpdRadPS + transInSpdRadPS);
end

% Calculate Torque Out of the Engine through the transmission   
    transTorqInNM = engTorq2TransNM*(~clutchIsSlip) + torqClutchNM*clutchIsSlip;
    transTorqOutNM = transTorqInNM*GR;
    
%% Modeling of a Differential Steering Mechanism
    steerMotorTorqNM = engTorq2StrMotorNM;
    steerMotorSpeedRadSec = 0.5*FD*abs(wl-wr);
    sprocketTorqueLeftNM = (1/2)*FD*transTorqOutNM + 0.5*FD*steerMotorTorqNM;
    sprocketTorqueRightNM = (1/2)*FD*transTorqOutNM - 0.5*FD*steerMotorTorqNM;
    Pin = steerMotorTorqNM*steerMotorSpeedRadSec + transTorqOutNM*transOutSpdRadPS;
    Pout = sprocketTorqueLeftNM*wl + sprocketTorqueRightNM*wr;   
    TL = sprocketTorqueLeftNM*(sprocketTorqueLeftNM > 0); 
    TR = sprocketTorqueRightNM*(sprocketTorqueRightNM > 0);
    
%% Calculate derivatives to propogate state 1 time step forward
make_M_F();      

    % Body States
    XDot = vx;
    YDot = 0;
    thetaDot = 0;
    vxd = a(1);
    vyd = 0;
    rd =  0;
    spwld = (TL -(Fl*rs) - trackDamp*wl)/Js ;
    spwrd = (TR -(Fr*rs) - trackDamp*wr)/Js ;
        nBodyState = [XDot YDot thetaDot vxd vyd rd spwld spwrd];

    % Powertrain States
    engThrtlDot = (1/transTau)*(-engThrtl + commandThrottle);
    engSpdRadPSDot;
    clutchEngageDot = (1/clutchTau)*(-com + clutchCmd);
    engineControlThrottleDot = (0.5*( 1-tanh(4*((engSpdRadPS - idleSpeedReferenceRadPS )/speedThresholdRadPS))) - engCtrlThrtl )/timeConstantIdleController;
        nPowerTrainState = [engThrtlDot engSpdRadPSDot clutchEngageDot engineControlThrottleDot];

    % Added States for Winch
    XDotsled = vXsled;
    vXDotsled = a(3);
    psiWinchRadDot = psiWinchRadPS;
    psiWinchRadPSDot = a(2);
    hydPressureHDot = (B/VH)*Qh;
    hydPressureODot = (B/VO)*Qo;
        addState = [XDotsled vXDotsled psiWinchRadDot psiWinchRadPSDot hydPressureHDot hydPressureODot];

    % Combined State Vector
    xdot(:,1) = real([nBodyState nPowerTrainState addState]');
   
%% Put other values of interest into data structure
    if strcmp(outputCond,'else')
        % Other
        MT865.forces = [Fl,Fr,RL,RR,RlLF,RlRF,RlLR,RlRR,sinkageLeftDynamic,sinkageRightDynamic].' ;
        MT865.slip = [slipLeft slipRight].';
        % Transmission
        MT865.transOutputSpeedRadSec = transOutSpdRadPS;
        MT865.transTorqueOutputNM = transTorqOutNM;
        MT865.transTorqueInputNM = transTorqInNM;
        MT865.steerMotorTorqueNM = steerMotorTorqNM;
        MT865.torqueClutchNM = torqClutchNM;
        
        % AG Hydraulics
        MT865.displacementPumpM3 = Dp;
        MT865.engTorq2AgPumpNM = engTorq2AgPumpNM;
        MT865.torqWinchNM = -torqWinchNM;
        MT865.FlowH = Qh;
        MT865.FlowO = Qo;
                
        % Derivatives and Drawbar Force
        MT865.xdot = xdot;
        MT865.drawBarPullN = a(4);

        % Write MT865 structure to output
        output = MT865;
        
    elseif strcmp(outputCond,'xdot')
        output = xdot;
        
    end

%% Child Function: Make M and F
    % This function Creates the M Matrix and F Vector
    function make_M_F()
        if ~winchIsLocked % Corresponds to pulling in Sled
            M = [mT 0  0   1;
                 0 Jw  0 -rw;
                 0  0 mS  -1;
                -1 rw  1   0];
            F(1,1) = Fl + Fr - RL - RR;
            F(2,1) = -torqWinchNM - sign(psiWinchRadPS)*winchDamp*psiWinchRadPS;
            F(3,1) = -RsledN;
            F(4,1) = 0;
            a = M\F;
        else % Winch is Locked
            M = [mT 0   1;
                 0  mS -1;
                -1  1  0];
            F(1,1) = Fl + Fr - RL - RR;
            F(2,1) = -RsledN;
            F(3,1) = 0;
            aHold = M\F; a(1) = aHold(1); a(2) = 0; 
                         a(3) = aHold(2); a(4) = aHold(3);
        end
    end

end



