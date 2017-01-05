function [output] = state_derivatives_MB_tractor(x, MT865,input,nConstantMT865,outputCond)
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
SGR = nConstantMT865.steerGearRatio;

transTau = nConstantMT865.timeConstantTransmission;
clutchTau = nConstantMT865.timeConstantClutch;
transDamp = nConstantMT865.transmissionDamp;
TfMaxs = nConstantMT865.clutchStaticFrictionCapNM;
TfMaxd = nConstantMT865.clutchDynamicFrictionCapNM;
Jtrans = nConstantMT865.inertiaTransmissionKGM2;
Jeng = nConstantMT865.inertiaEngineKGM2;
engDamp = nConstantMT865.engineDamp;
steerAngleTau = nConstantMT865.steerAngleTau;

lArm = nConstantMT865.towArmM;
mSled = nConstantMT865.mSledKG;
lSled = nConstantMT865.sledLengthM;
TCG2H = nConstantMT865.TTCG2Hitch;
JSled = nConstantMT865.inertiaSledKGM2;
lTool = nConstantMT865.CRLTool;
RsledN = nConstantMT865.RsledN;

Vh = nConstantMT865.volumehM3;
DpMax = nConstantMT865.DpMaxM3;
Dm = nConstantMT865.DmM3;
B = nConstantMT865.bulkModulus;
GRp = nConstantMT865.GRpump;
GRst = nConstantMT865.GRStrMotor;


% massMatrix = nConstantMT865.massMatrix;
% forceVector = nConstantMT865.forceVector;
%% Unwrap Inputs and State

% Inputs
commandThrottle = input(1);
gear = input(2);
%steerAngleDeg = input(3);
steerPumpCmd = input(3); % Range from 1 to -1, replaces steerAngleDeg
clutchCmd = input(4);
GR = nGR(gear);

% Body State
X = x(1);
Y = x(2);
theta = x(3);
phi = x(4);
si = x(5);
vx = x(6)*(x(6) > 0); % Logic Binds state to be equal or greater than 0
vy = x(7);
dtheta = x(8);
dphi = x(9);
dsi = x(10);
wl = x(11)*(x(11) > 0); % Logic Binds state to be equal or greater than 0
wr = x(12)*(x(12) > 0); % Logic Binds state to be equal or greater than 0


% PowerTrain State
engThrtl = x(13); % Transmission Output Toruqe
engSpdRadPS = x(14); % Engine Speed State
com = x(15);   % Clutch Engage Cmd
engCtrlThrtl = x(16); 
steerAngleState = x(17);
pressureHPa = x(18);

% Unwrap Current State (In the Sense of a State Machine)
clutchIsSlip = MT865.clutchIsSlip;
RsledN = RsledN*(sqrt(vx^2 + vy^2) > 0);


%% Attempt at Control of Throttle for max speed
    commandThrottle = commandThrottle*tanh( abs(engSpdRadPS - (2100*((2*pi)/60))) / 10); 

%% Calculate Torque out of the engine and how it is divided between the transmission and steering motor
    engTorqNM = engine_interp( engThrtl, engCtrlThrtl, engSpdRadPS, nConstantMT865 );
    %engTorq2StrPumpNM = (steerAngleState/170)*DpMax*pressureHPa;
    engTorq2StrPumpNM = abs( steerPumpCmd*DpMax*pressureHPa );
    engTorq2TransNM = engTorqNM - engTorq2StrPumpNM;
    
%% Calculate Forces at the Track
[Fl,Fr,RL,RR,RlLF,RlRF,RlLR,RlRR,slipLeft,slipRight] = track_forces(x, MT865, nConstantMT865);

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
    %Qh = ((steerAngleState/170)*DpMax*engSpdRadPS*GRp ) - Dm*GRst*(wr-wl);
    Qh = (steerPumpCmd*DpMax*engSpdRadPS*GRp ) - Dm*GRst*(wr-wl);
    
    sprocketTorqueLeftNM = (1/2)*FD*transTorqOutNM - (1/2)*GRst*Dm*pressureHPa;
    sprocketTorqueRightNM = (1/2)*FD*transTorqOutNM + (1/2)*GRst*Dm*pressureHPa;
  
    TL = sprocketTorqueLeftNM*(sprocketTorqueLeftNM > 0); 
    TR = sprocketTorqueRightNM*(sprocketTorqueRightNM > 0);
    
%% Calculate Sled Resistance
    R3_1 = rotation_matrix_z(phi); % frame1 (tractor Frame) to frame 3 (Sled Frame)
    R3_2 = rotation_matrix_z(si - phi); % frame 2 (tow arm frame) to frame 3 (Sled Frame)
    vTractor = [vx; vy; 0];
    vA = R3_1*( vTractor + cross([0 0 dtheta].',[-TCG2H 0 0].') ); 
    vB = vA + R3_2*( cross([0 0 dphi].', [-lArm 0 0].') );
    vSled = vB + cross([0 0 dsi].', [-((lSled/2)+lTool), 0, 0].');
    mag_vSled = sqrt( vSled(1)^2 + vSled(2)^2 + vSled(3)^2 ); % L2 norm of vector
    if mag_vSled == 0
        RSledx3 = 0;
        RSledy3 = 0;
    else
        vSledDirection = atan2(vSled(2),vSled(1));
        RSledx3 = -RsledN*cos(vSledDirection);
        RSledy3 = -RsledN*sin(vSledDirection);
    end

%% Calculate derivatives to propogate state 1 time step forward
%     Msubd = double(subs(massMatrix));
%     Fsubd = double(subs(forceVector));
    [Msubd, Fsubd] = make_Msubd_Fsubd;
    x6_10dot = Msubd\Fsubd;
        % Body States
        XDot = vx*cos(theta) - vy*sin(theta);
        YDot = vx*sin(theta) + vy*cos(theta);
        dtheta = dtheta;
        dphi = dphi;
        dsi = dsi;
                
        dvx = x6_10dot(1);
        dvy = x6_10dot(2);
        d2theta = x6_10dot(3);
        d2phi = x6_10dot(4);
        d2si = x6_10dot(5);

        spwld = (TL - (Fl*rs) - trackDamp*wl)/Js ;
        spwrd = (TR - (Fr*rs) - trackDamp*wr)/Js ;
            nBodyStateDot = [XDot YDot dtheta dphi dsi dvx dvy d2theta d2phi d2si spwld spwrd];

        % Powertrain States
        engThrtlDot = (1/transTau)*(-engThrtl + commandThrottle);
        engSpdRadPSDot;
        clutchEngageDot = (1/clutchTau)*(-com + clutchCmd);
        engineControlThrottleDot = (0.5*( 1-tanh(4*((engSpdRadPS - (1305*((2*pi)/60) ) )/5))) - engCtrlThrtl )/0.5;
        %steerAngleStateDot = (1/steerAngleTau)*(-steerAngleState + steerAngleDeg);
        steerAngleStateDot = 0;
        pressureHPaDot = (B/Vh)*Qh;
            nPowerTrainStateDot = [engThrtlDot engSpdRadPSDot clutchEngageDot engineControlThrottleDot steerAngleStateDot pressureHPaDot];

        % Combined State Vector
        xdot(:,1) = real([nBodyStateDot nPowerTrainStateDot]');
    if strcmp(outputCond,'xdot')
        output = xdot;
    end
    
%% Put other values of interest into data structure
    if strcmp(outputCond,'else')
        MT865.forces = [Fl,Fr,RL,RR,RlLF,RlRF,RlLR,RlRR].' ;
        MT865.slip = [slipLeft slipRight].';
        MT865.transOutputSpeedRadSec = transOutSpdRadPS;
        MT865.transTorqueOutputNM = transTorqOutNM;
        MT865.transTorqueInputNM = transTorqInNM;
        MT865.steerMotorTorqueNM = engTorq2StrPumpNM;
        MT865.torqueClutchNM = torqClutchNM;
        MT865.torqueRightSprocketNM = TR;
        MT865.torqueLeftSprocketNM = TL;
        MT865.xdot = xdot(:,1);
        MT865.sledForcexy = [RSledx3 RSledy3].';
        MT865.vSledxy = [vSled(1) vSled(2) vSled(3)].';
        MT865.enginePowerW = engTorqNM*engSpdRadPS;

        output = MT865;
    end
         
%% Make M and F

%     function [Msubd, Fsubd] = make_Msubd_Fsubd
%         Msubd = [                                                    mSled + mT,                                                                        0,                                                                                                                            (mSled*(2*lArm*sin(phi) + lSled*sin(si) + 2*lTool*sin(si)))/2,                                                                      lArm*mSled*sin(phi),                                                    (mSled*sin(si)*(lSled + 2*lTool))/2;
%                                                                              0,                                                               mSled + mT,                                                                                                                 -(mSled*(2*TCG2H + 2*lArm*cos(phi) + lSled*cos(si) + 2*lTool*cos(si)))/2,                                                                     -lArm*mSled*cos(phi),                                                   -(mSled*cos(si)*(lSled + 2*lTool))/2;
%                  (mSled*(2*lArm*sin(phi) + lSled*sin(si) + 2*lTool*sin(si)))/2, -(mSled*(2*TCG2H + 2*lArm*cos(phi) + lSled*cos(si) + 2*lTool*cos(si)))/2, JT + (mSled*(2*(lArm*cos(phi + theta) + TCG2H*cos(theta) + cos(si + theta)*(lSled/2 + lTool))^2 + 2*(lArm*sin(phi + theta) + TCG2H*sin(theta) + sin(si + theta)*(lSled/2 + lTool))^2))/2, (lArm*mSled*(2*lArm + 2*TCG2H*cos(phi) + lSled*cos(phi - si) + 2*lTool*cos(phi - si)))/2, (mSled*(lSled/2 + lTool)*(lSled + 2*lTool + 2*TCG2H*cos(si) + 2*lArm*cos(phi - si)))/2;
%                                                            lArm*mSled*sin(phi),                                                     -lArm*mSled*cos(phi),                                                                                                 (lArm*mSled*(2*lArm + 2*TCG2H*cos(phi) + lSled*cos(phi - si) + 2*lTool*cos(phi - si)))/2,                                                                             lArm^2*mSled,                                         (lArm*mSled*cos(phi - si)*(lSled + 2*lTool))/2;
%                                            (mSled*sin(si)*(lSled + 2*lTool))/2,                                     -(mSled*cos(si)*(lSled + 2*lTool))/2,                                                                                                   (mSled*(lSled/2 + lTool)*(lSled + 2*lTool + 2*TCG2H*cos(si) + 2*lArm*cos(phi - si)))/2,                                           (lArm*mSled*cos(phi - si)*(lSled + 2*lTool))/2,                          (mSled*lSled^2)/4 + mSled*lSled*lTool + mSled*lTool^2 + JSled];
% 
%         Fsubd = [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       Fl + Fr - RL - RR + RSledx3*cos(si) - RSledy3*sin(si) + dtheta*mSled*vy + dtheta*mT*vy - TCG2H*dtheta^2*mSled - dphi^2*lArm*mSled*cos(phi) - dtheta^2*lArm*mSled*cos(phi) - (dsi^2*lSled*mSled*cos(si))/2 - (dtheta^2*lSled*mSled*cos(si))/2 - dsi^2*lTool*mSled*cos(si) - dtheta^2*lTool*mSled*cos(si) - 2*dphi*dtheta*lArm*mSled*cos(phi) - dsi*dtheta*lSled*mSled*cos(si) - 2*dsi*dtheta*lTool*mSled*cos(si);
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      RlLF + RlLR + RlRF + RlRR + RSledy3*cos(si) + RSledx3*sin(si) - dtheta*mSled*vx - dtheta*mT*vx - dphi^2*lArm*mSled*sin(phi) - dtheta^2*lArm*mSled*sin(phi) - (dsi^2*lSled*mSled*sin(si))/2 - (dtheta^2*lSled*mSled*sin(si))/2 - dsi^2*lTool*mSled*sin(si) - dtheta^2*lTool*mSled*sin(si) - 2*dphi*dtheta*lArm*mSled*sin(phi) - dsi*dtheta*lSled*mSled*sin(si) - 2*dsi*dtheta*lTool*mSled*sin(si);
%                  (Fr*bT)/2 - (Fl*bT)/2 + (RL*bT)/2 - (RR*bT)/2 - (RSledy3*lSled)/2 - RSledy3*lTool + (LT*RlLF)/4 - (LT*RlLR)/4 + (LT*RlRF)/4 - (LT*RlRR)/4 - RSledy3*lArm*cos(phi - si) + RSledx3*lArm*sin(phi - si) - RSledy3*TCG2H*cos(si) - RSledx3*TCG2H*sin(si) + TCG2H*dtheta*mSled*vx + TCG2H*dphi^2*lArm*mSled*sin(phi) + (TCG2H*dsi^2*lSled*mSled*sin(si))/2 + TCG2H*dsi^2*lTool*mSled*sin(si) + (dphi^2*lArm*lSled*mSled*sin(phi - si))/2 + dphi^2*lArm*lTool*mSled*sin(phi - si) - (dsi^2*lArm*lSled*mSled*sin(phi - si))/2 - dsi^2*lArm*lTool*mSled*sin(phi - si) + dtheta*lArm*mSled*vx*cos(phi) + (dtheta*lSled*mSled*vx*cos(si))/2 + dtheta*lTool*mSled*vx*cos(si) + dtheta*lArm*mSled*vy*sin(phi) + (dtheta*lSled*mSled*vy*sin(si))/2 + dtheta*lTool*mSled*vy*sin(si) + 2*TCG2H*dphi*dtheta*lArm*mSled*sin(phi) + TCG2H*dsi*dtheta*lSled*mSled*sin(si) + 2*TCG2H*dsi*dtheta*lTool*mSled*sin(si) + dphi*dtheta*lArm*lSled*mSled*sin(phi - si) + 2*dphi*dtheta*lArm*lTool*mSled*sin(phi - si) - dsi*dtheta*lArm*lSled*mSled*sin(phi - si) - 2*dsi*dtheta*lArm*lTool*mSled*sin(phi - si);
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -(lArm*(2*RSledy3*cos(phi - si) - 2*RSledx3*sin(phi - si) + dsi^2*lSled*mSled*sin(phi - si) + dtheta^2*lSled*mSled*sin(phi - si) + 2*dsi^2*lTool*mSled*sin(phi - si) + 2*dtheta^2*lTool*mSled*sin(phi - si) - 2*dtheta*mSled*vx*cos(phi) - 2*dtheta*mSled*vy*sin(phi) + 2*TCG2H*dtheta^2*mSled*sin(phi) + 2*dsi*dtheta*lSled*mSled*sin(phi - si) + 4*dsi*dtheta*lTool*mSled*sin(phi - si)))/2;
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     ((lSled + 2*lTool)*(dphi^2*lArm*mSled*sin(phi - si) - RSledy3 + dtheta^2*lArm*mSled*sin(phi - si) + dtheta*mSled*vx*cos(si) + dtheta*mSled*vy*sin(si) - TCG2H*dtheta^2*mSled*sin(si) + 2*dphi*dtheta*lArm*mSled*sin(phi - si)))/2];
%     end

    function [Msubd, Fsubd] = make_Msubd_Fsubd
         Msubd = [                                                    mSled + mT,                                                                        0,                                                                                                                                                                                                 (mSled*(2*lArm*sin(phi) + lSled*sin(si) + 2*lTool*sin(si)))/2,                                                                      lArm*mSled*sin(phi),                                                                                                                                                             (mSled*sin(si)*(lSled + 2*lTool))/2;
                                                                                     0,                                                               mSled + mT,                                                                                                                                                                                      -(mSled*(2*TCG2H + 2*lArm*cos(phi) + lSled*cos(si) + 2*lTool*cos(si)))/2,                                                                     -lArm*mSled*cos(phi),                                                                                                                                                            -(mSled*cos(si)*(lSled + 2*lTool))/2;
                         (mSled*(2*lArm*sin(phi) + lSled*sin(si) + 2*lTool*sin(si)))/2, -(mSled*(2*TCG2H + 2*lArm*cos(phi) + lSled*cos(si) + 2*lTool*cos(si)))/2, mSled*TCG2H^2 + 2*mSled*cos(phi)*TCG2H*lArm + mSled*cos(si)*TCG2H*lSled + 2*mSled*cos(si)*TCG2H*lTool + mSled*lArm^2 + mSled*cos(phi - si)*lArm*lSled + 2*mSled*cos(phi - si)*lArm*lTool + (mSled*lSled^2)/4 + mSled*lSled*lTool + mSled*lTool^2 + JSled + JT, (lArm*mSled*(2*lArm + 2*TCG2H*cos(phi) + lSled*cos(phi - si) + 2*lTool*cos(phi - si)))/2, JSled + (lSled^2*mSled)/4 + lTool^2*mSled + lSled*lTool*mSled + (TCG2H*lSled*mSled*cos(si))/2 + TCG2H*lTool*mSled*cos(si) + (lArm*lSled*mSled*cos(phi - si))/2 + lArm*lTool*mSled*cos(phi - si);
                                                                   lArm*mSled*sin(phi),                                                     -lArm*mSled*cos(phi),                                                                                                                                                                      (lArm*mSled*(2*lArm + 2*TCG2H*cos(phi) + lSled*cos(phi - si) + 2*lTool*cos(phi - si)))/2,                                                                             lArm^2*mSled,                                                                                                                                                  (lArm*mSled*cos(phi - si)*(lSled + 2*lTool))/2;
                                                  (mSled*sin(si)*(lSled + 2*lTool))/2,                                     -(mSled*cos(si)*(lSled + 2*lTool))/2,                                                               JSled + (lSled^2*mSled)/4 + lTool^2*mSled + lSled*lTool*mSled + (TCG2H*lSled*mSled*cos(si))/2 + TCG2H*lTool*mSled*cos(si) + (lArm*lSled*mSled*cos(phi - si))/2 + lArm*lTool*mSled*cos(phi - si),                                           (lArm*mSled*cos(phi - si)*(lSled + 2*lTool))/2,                                                                                                                                   (mSled*lSled^2)/4 + mSled*lSled*lTool + mSled*lTool^2 + JSled];
    
        Fsubd =  [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      Fl + Fr - RL - RR + RSledx3*cos(si) - RSledy3*sin(si) + dtheta*mSled*vy + dtheta*mT*vy - TCG2H*dtheta^2*mSled - dphi^2*lArm*mSled*cos(phi) - dtheta^2*lArm*mSled*cos(phi) - (dsi^2*lSled*mSled*cos(si))/2 - (dtheta^2*lSled*mSled*cos(si))/2 - dsi^2*lTool*mSled*cos(si) - dtheta^2*lTool*mSled*cos(si) - 2*dphi*dtheta*lArm*mSled*cos(phi) - dsi*dtheta*lSled*mSled*cos(si) - 2*dsi*dtheta*lTool*mSled*cos(si);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     RlLF + RlLR + RlRF + RlRR + RSledy3*cos(si) + RSledx3*sin(si) - dtheta*mSled*vx - dtheta*mT*vx - dphi^2*lArm*mSled*sin(phi) - dtheta^2*lArm*mSled*sin(phi) - (dsi^2*lSled*mSled*sin(si))/2 - (dtheta^2*lSled*mSled*sin(si))/2 - dsi^2*lTool*mSled*sin(si) - dtheta^2*lTool*mSled*sin(si) - 2*dphi*dtheta*lArm*mSled*sin(phi) - dsi*dtheta*lSled*mSled*sin(si) - 2*dsi*dtheta*lTool*mSled*sin(si);
                         (Fr*bT)/2 - (Fl*bT)/2 + (RL*bT)/2 - (RR*bT)/2 - (RSledy3*lSled)/2 - RSledy3*lTool + (LT*RlLF)/4 - (LT*RlLR)/4 + (LT*RlRF)/4 - (LT*RlRR)/4 - RSledy3*lArm*cos(phi - si) + RSledx3*lArm*sin(phi - si) - RSledy3*TCG2H*cos(si) - RSledx3*TCG2H*sin(si) + TCG2H*dtheta*mSled*vx + TCG2H*dphi^2*lArm*mSled*sin(phi) + (TCG2H*dsi^2*lSled*mSled*sin(si))/2 + TCG2H*dsi^2*lTool*mSled*sin(si) + (dphi^2*lArm*lSled*mSled*sin(phi - si))/2 + dphi^2*lArm*lTool*mSled*sin(phi - si) - (dsi^2*lArm*lSled*mSled*sin(phi - si))/2 - dsi^2*lArm*lTool*mSled*sin(phi - si) + dtheta*lArm*mSled*vx*cos(phi) + (dtheta*lSled*mSled*vx*cos(si))/2 + dtheta*lTool*mSled*vx*cos(si) + dtheta*lArm*mSled*vy*sin(phi) + (dtheta*lSled*mSled*vy*sin(si))/2 + dtheta*lTool*mSled*vy*sin(si) + 2*TCG2H*dphi*dtheta*lArm*mSled*sin(phi) + TCG2H*dsi*dtheta*lSled*mSled*sin(si) + 2*TCG2H*dsi*dtheta*lTool*mSled*sin(si) + dphi*dtheta*lArm*lSled*mSled*sin(phi - si) + 2*dphi*dtheta*lArm*lTool*mSled*sin(phi - si) - dsi*dtheta*lArm*lSled*mSled*sin(phi - si) - 2*dsi*dtheta*lArm*lTool*mSled*sin(phi - si);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        -(lArm*(2*RSledy3*cos(phi - si) - 2*RSledx3*sin(phi - si) + dsi^2*lSled*mSled*sin(phi - si) + dtheta^2*lSled*mSled*sin(phi - si) + 2*dsi^2*lTool*mSled*sin(phi - si) + 2*dtheta^2*lTool*mSled*sin(phi - si) - 2*dtheta*mSled*vx*cos(phi) - 2*dtheta*mSled*vy*sin(phi) + 2*TCG2H*dtheta^2*mSled*sin(phi) + 2*dsi*dtheta*lSled*mSled*sin(phi - si) + 4*dsi*dtheta*lTool*mSled*sin(phi - si)))/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ((lSled + 2*lTool)*(dphi^2*lArm*mSled*sin(phi - si) - RSledy3 + dtheta^2*lArm*mSled*sin(phi - si) + dtheta*mSled*vx*cos(si) + dtheta*mSled*vy*sin(si) - TCG2H*dtheta^2*mSled*sin(si) + 2*dphi*dtheta*lArm*mSled*sin(phi - si)))/2];                                       
    end

end

function R = rotation_matrix_z(angle)
    R = [cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];
end


