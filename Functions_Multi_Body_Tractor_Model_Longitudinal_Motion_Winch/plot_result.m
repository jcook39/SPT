function plot_result(tractor,inputMat,lineStyle,nConstantMT865,nConstantTerrain,nTimeParam,plotContour)

%% Unpack Constants
trackAreaM2 = nConstantMT865.trackAreaM2;
normalForceTrackN = nConstantMT865.normalForceTrackN;
RsledN = nConstantMT865.RsledN;

%% Create time Vector for plotting
nTimeStep = nTimeParam.nTimeStep;
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;

indexVector = 1:nTimeStep;
timeVector = (indexVector-1)*timeStepS;

%% Declare Vectors for States
initVec = zeros(nTimeStep,1);
X = zeros(nTimeStep,1);
Y = zeros(nTimeStep,1);
theta = zeros(nTimeStep,1);
vx = zeros(nTimeStep,1);
vy = zeros(nTimeStep,1);
thetaDot = zeros(nTimeStep,1);
wl = zeros(nTimeStep,1);
wr = zeros(nTimeStep,1);
engThrtlState = zeros(nTimeStep,1);
engSpdRadPS = zeros(nTimeStep,1);
clutchComState = zeros(nTimeStep,1);
engCtrlThrtl = zeros(nTimeStep,1);

%% Declare Vectors for Other Data
clutchIsSlip = initVec;

%% Unpack Inputs
throttle = inputMat(indexVector,1);
gear = inputMat(indexVector,2);
steerAngleDeg = inputMat(indexVector,3);
clutchCmd = inputMat(indexVector,4);
valvePos = inputMat(indexVector,5);


for i = 1:nTimeStep
    % State Vector
    X(i) = tractor(i).state(1);
    Y(i) = tractor(i).state(2);
    theta(i) = tractor(i).state(3);
    vx(i) = tractor(i).state(4);
    vy(i) = tractor(i).state(5);
    thetaDot(i) = tractor(i).state(6);
    wl(i) = tractor(i).state(7);
    wr(i) = tractor(i).state(8);
    engThrtlState(i) = tractor(i).state(9);
    engSpdRadPS(i) = tractor(i).state(10);
    clutchComState(i) = tractor(i).state(11);
    engCtrlThrtl(i) = tractor(i).state(12);
    Xsled(i) = tractor(i).state(13);
    vXsled(i) = tractor(i).state(14);
    psiWinchRad(i) = tractor(i).state(15);
    psiWinchRadPS(i) = tractor(i).state(16);
    hydPressureH(i) = tractor(i).state(17);
    hydPressureO(i) = tractor(i).state(18);
    % Other
    slipLeft(i) = tractor(i).slip(1);
    slipRight(i) = tractor(i).slip(2);
    % Transmisison 
    clutchIsSlip(i) = tractor(i).clutchIsSlip;
    % AG Hydraulics
    Dp(i) = tractor(i).displacementPumpM3;
    engTorq2AgPumpNM(i) = tractor(i).engTorq2AgPumpNM;
    torqWinchNM(i) = tractor(i).torqWinchNM;
    FlowH(i) = tractor(i).FlowH;
    FlowO(i) = tractor(i).FlowO;
    
    % Forces
    DBP(i) = tractor(i).drawBarPullN;
    Fl(i) = tractor(i).forces(1);
    Fr(i) = tractor(i).forces(2);
    RL(i) = tractor(i).forces(3);
    RR(i) = tractor(i).forces(4);
    
    %Terrain Parameters
    terrainCohesion(i) = tractor(i).terrainLeftFront(1);
    terrainFrictionAngle(i) = tractor(i).terrainLeftFront(2);
    terrainK(i) = tractor(i).terrainLeftFront(3);
    terrainkeq(i) = tractor(i).terrainLeftFront(4);
    terrainn(i) = tractor(i).terrainLeftFront(5);
    terrainS(i) = tractor(i).terrainLeftFront(6);
    
    
    % Controller Parameters
    Error(i) = tractor(i).Error;
    intError(i) = tractor(i).intError;
    pSetBrakeValve(i) = tractor(i).pSetBrakeValve;
end

%% Unpack Constant Parameters
DpMax = nConstantMT865.displacementPumpM3;
rw = nConstantMT865.winchRadiusM;
b = nConstantMT865.trackCG2trackCG;
l = nConstantMT865.trackLengthM;

%% Unpack Terrain Plat
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
contourX = nConstantTerrain.X;
contourY = nConstantTerrain.Y;
frictionAngle = nConstantTerrain.terrainFrictionAngle;
cohesion = nConstantTerrain.terrainCohesion;

%% Spatial Plots
timeStepVehicle = 10; % seconds
incVehicle = timeStepVehicle/timeStepS;

figure(1)
if plotContour
contourf(contourX,contourY,frictionAngle,'edgecolor','none')
caxis([0 30])
colormap winter
%colorbar('EastOutside')
end
hold on
plot(X,Y,lineStyle)
hold on
plot(X,Y,lineStyle)
hold on
dim1 = [0.2 0.3 0.28 0.57];
str1 = 'Terrain 1';
annotation('textbox',dim1,'String',str1,'FitBoxToText','on','linestyle','none','fontsize',16);
dim2 = [0.6 0.3 0.4 0.57];
str2 = 'Terrain 2';
annotation('textbox',dim2,'String',str2,'FitBoxToText','on','linestyle','none','fontsize',16);

tractorColor = lineStyle;
for i=1:incVehicle:nTimeStep
  
    plot_tractor_on_map_longitudinal(tractor,i,nConstantMT865,tractorColor)
    
%   cosTheta = cos(theta(i));
%   sinTheta = sin(theta(i));
%   p1X = X(i) + l/2*cosTheta - b*sinTheta/2;
%   p2X = X(i) + l/2*cosTheta + b*sinTheta/2;
%   p3X = X(i) - l/2*cosTheta + b*sinTheta/2;
%   p4X = X(i) - l/2*cosTheta - b*sinTheta/2;
%   p1Y = Y(i) + l/2*sinTheta + b*cosTheta/2;
%   p2Y = Y(i) + l/2*sinTheta - b*cosTheta/2;
%   p3Y = Y(i) - l/2*sinTheta - b*cosTheta/2;
%   p4Y = Y(i) - l/2*sinTheta + b*cosTheta/2;
%   plot([p1X p2X p3X p4X p1X],[p1Y p2Y p3Y p4Y p1Y],lineStyle);
%   plot(X(i),Y(i),lineStyle);   % cg
%   hold on
  
end;
%title('Friction Angle of Terrain')
ylabel('North Position (meters)','interpreter','latex','fontsize',16)
xlabel('East Position (meters)','interpreter','latex','fontsize',16)
xlim([0 gridSizeXM])
ylim([0 gridSizeYM])
hold on

figure(2)
if plotContour
contourf(contourX,contourY,cohesion,'edgecolor','none')
caxis([0 30])
colormap gray
colorbar('EastOutside')
end
hold on
plot(X,Y,lineStyle);
hold on
plot(X,Y,lineStyle);
hold on

for i=1:incVehicle:nTimeStep
  
  cosTheta = cos(theta(i));
  sinTheta = sin(theta(i));
  p1X = X(i) + l/2*cosTheta - b*sinTheta/2;
  p2X = X(i) + l/2*cosTheta + b*sinTheta/2;
  p3X = X(i) - l/2*cosTheta + b*sinTheta/2;
  p4X = X(i) - l/2*cosTheta - b*sinTheta/2;
  p1Y = Y(i) + l/2*sinTheta + b*cosTheta/2;
  p2Y = Y(i) + l/2*sinTheta - b*cosTheta/2;
  p3Y = Y(i) - l/2*sinTheta - b*cosTheta/2;
  p4Y = Y(i) - l/2*sinTheta + b*cosTheta/2;
  plot([p1X p2X p3X p4X p1X],[p1Y p2Y p3Y p4Y p1Y],lineStyle);
  plot(X(i),Y(i),lineStyle);   % cg
  hold on
  
end;
title('Cohesion of Terrain')
ylabel('lateral position (m)')
xlabel('longitudinal position (m)')
xlim([0 200])
ylim([0 100])
hold on

%% Data Plots
figure(3)
subplot(421)
    plot(timeVector,X(indexVector),lineStyle,timeVector,20*ones(numel(indexVector),1),timeVector,35*ones(numel(indexVector),1))
    ylabel('X Pos (m)')
    hold on
subplot(422)
    plot(timeVector,Y(indexVector),lineStyle)
    ylabel('Y Pos (m)')
    hold on
subplot(423)
    plot(timeVector,theta(indexVector),lineStyle)
    ylabel('yaw Angle ')
    hold on
subplot(424)
    plot(timeVector,throttle(indexVector),timeVector,engThrtlState,lineStyle,timeVector,engCtrlThrtl(indexVector))
    legend('Throttle Cmd','Throttle Cmd State','Eng Ctrl Thrtl')
    ylim([0 1])
    hold on
subplot(425)
    plot(timeVector,gear(indexVector),lineStyle)
    ylabel('gear Selection')
    hold on
subplot(426)
    plot(timeVector,steerAngleDeg(indexVector),lineStyle)
    ylabel('steer Angle Deg')
    hold on
subplot(427)
    plot(timeVector,clutchComState,lineStyle)
    legend('Commanded Engagement','Actual Engagement')
    ylim([0 1])
    hold on
subplot(428)
    plot(timeVector,clutchIsSlip(indexVector),lineStyle)
    ylabel('Clutch is Slipping = 1')
    hold on
    
figure(4)
subplot(421)
    plot(timeVector,vx(indexVector),lineStyle)
    ylabel('vx (m/s)') 
    hold on
subplot(422)
    plot(timeVector,vy(indexVector),lineStyle)
    ylabel('vy (m/s)') 
    hold on
subplot(423)
    plot(timeVector,thetaDot(indexVector),lineStyle)
    ylabel('thetaDot (rad/s)')   
    hold on
subplot(424)
    plot(timeVector,wl(indexVector),lineStyle)
    ylabel('wl (rad/s)')   
    hold on
subplot(425)
    plot(timeVector,wr(indexVector),lineStyle)
    ylabel('wl (rad/s)')   
    hold on
subplot(426)
    plot(timeVector,engThrtlState(indexVector),lineStyle)
    ylabel('engine Torque (Nm)')
    hold on
subplot(427)
    plot(timeVector,engSpdRadPS(indexVector).*((60)/(2*pi)),lineStyle)
    ylim([1000 2300])
    ylabel('eng Speed (RPM)')
    hold on
    
figure(5)
subplot(421)
    plot(timeVector,Xsled(indexVector),lineStyle)
    ylabel('X Pos Sled')
    hold on
subplot(422)
    plot(timeVector,vXsled(indexVector),lineStyle)
    ylabel('Speed Sled')
    hold on
subplot(423)
    plot(timeVector,psiWinchRad(indexVector)*rw,lineStyle)
    ylabel('Winch Pos (m)')
    hold on
subplot(424)
    plot(timeVector,psiWinchRadPS(indexVector)*rw,lineStyle)
    ylabel('Winch Speed (m/s)')
    hold on
subplot(425)
    plot(timeVector,hydPressureH(indexVector)./6894.76,lineStyle)
    ylabel('Pressure H psi')
    ylim([0 2900])
    hold on
subplot(426)
    plot(timeVector,hydPressureO(indexVector)./6894.76,lineStyle)
    legend('Pressure Braking')
    ylabel('Pressure O psi')
    hold on
    %ylim([0 2900])
subplot(427)
    plot(timeVector,Dp(indexVector)/DpMax,lineStyle)
%    ylim([0 DpMax]
    ylabel('Pump Displacement')
    hold on
subplot(428)
    plot(timeVector,valvePos(indexVector),lineStyle)
    ylabel('Valve Position')
    hold on
    
figure(6)
subplot(121)
    plot(timeVector,slipLeft(indexVector),lineStyle)
    ylim([0 100])
    ylabel('Slip Ratio Left')
    hold on
subplot(122)
    plot(timeVector,slipRight(indexVector),lineStyle)
    ylim([0 100])
    ylabel('Slip Ratio Left') 
    hold on
    
figure(7)
subplot(421)
    plot(timeVector,DBP(indexVector),lineStyle,timeVector,RsledN(ones(numel(indexVector),1)) )
     legend('Tow Force (N)','Total Resistance Force','Sled Friction force')
    %ylim([0 max(DBP(indexVector)+RL(indexVector)+RR(indexVector))])
    hold on
subplot(422)
    plot(timeVector,torqWinchNM(indexVector),lineStyle)
    ylabel('Torque On Winch (N-m)')
    hold on
subplot(423)
    plot(timeVector,engTorq2AgPumpNM(indexVector),lineStyle)
    ylabel('Pump Load on Engine (N-m)')
    hold on
subplot(424)
    plot(timeVector,intError(indexVector),lineStyle)
    ylabel('integral Error')
    hold on
subplot(425)
    plot(timeVector,Error(indexVector),lineStyle)
    ylabel('error')
    hold on
subplot(426)
    plot(timeVector,FlowH(indexVector),lineStyle)
    ylabel('Flow m^3/s H Node')
    hold on
subplot(427)
    plot(timeVector,FlowO(indexVector),lineStyle)
    ylabel('Flow m^3/s O Node')
    hold on
subplot(428)
    FmaxN = 2*(trackAreaM2.*terrainCohesion + (normalForceTrackN/1000).*tand(terrainFrictionAngle))*1000;
    plot(timeVector,Fl(indexVector)+Fr(indexVector),lineStyle,timeVector,FmaxN(indexVector),lineStyle)
    ylabel('Total Traction Effort (N)')
    hold on
    
figure(8)
subplot(321)
    plot(timeVector,terrainCohesion(indexVector),lineStyle)
    ylabel('cohesion')
    hold on
subplot(322)
    plot(timeVector,terrainFrictionAngle(indexVector),lineStyle)
    ylabel('friction angle (deg)')
    hold on
subplot(323)
    plot(timeVector,terrainK(indexVector),lineStyle)
    ylabel('Shear Def Modulus')
    hold on
subplot(324)
    plot(timeVector,terrainkeq(indexVector),lineStyle)
    ylabel('keq')
    hold on
subplot(325)
    plot(timeVector,terrainn(indexVector),lineStyle)
    ylabel('n')
    hold on
subplot(326)
    plot(timeVector,terrainS(indexVector),lineStyle)
    ylabel('S')
    hold on

    
% Calculate Engine Power
engTorqNM = zeros(nTimeStep,1);
for i = 1:nTimeStep
   engTorqNM(i) = engine_interp( engThrtlState(i), engCtrlThrtl(i), engSpdRadPS(i), nConstantMT865);
end
engPowerkW = (engTorqNM.*engSpdRadPS)/1000;

AX = [0.005 0.018];
font = 16;
figure(9)
if plotContour
subplot1(5,2,'Gap',AX,'YTickL','All','Min',[0.05 0.07],'Max',[0.95 0.99],'FontS',font)
end
subplot1(1)
    plot(timeVector,X(indexVector),lineStyle,timeVector,20*ones(numel(indexVector),1),':k',timeVector,35*ones(numel(indexVector),1),':k')
   ylabel('$X_T\hspace{2mm}(m)$','interpreter','latex','fontname','calibri','fontsize',font)
   ylim([10 60])
    hold on
subplot1(2)
    plot(timeVector,vx(indexVector),lineStyle)
    ylabel('$v_T\hspace{2mm}(m/s)$','interpreter','latex','fontname','calibri','fontsize',font) 
    set(gca,'yaxislocation','right');
    hold on
subplot1(3)
    plot(timeVector,psiWinchRad(indexVector)*rw,lineStyle)
    %ylabel('Winch Pos (m)','fontsize',font)
    ylabel('${\psi}r_W\hspace{2mm}(m)$','interpreter','latex','fontsize',font)
    ylim([0 25])
    hold on
subplot1(4)
    plot(timeVector,psiWinchRadPS(indexVector)*rw,lineStyle)
    %ylabel('Winch Speed (m/s)','fontsize',font) 
    ylabel('$\dot{\psi}r_W\hspace{2mm}(m/s)$','interpreter','latex','fontsize',font)
    set(gca,'yaxislocation','right');
    hold on
subplot1(5)
    plot(timeVector,vXsled(indexVector),lineStyle)
    ylabel('$v_S\hspace{2mm}(m/s)$','interpreter','latex','fontname','calibri','fontsize',font) 
    hold on
subplot1(6)
    plot(timeVector,DBP(indexVector)./1000,lineStyle)
    ylabel('$T\hspace{2mm}(kN)$','interpreter','latex','fontname','calibri','fontsize',font) 
    set(gca,'yaxislocation','right');
    ylim([0 200])
    hold on
subplot1(7)
    plot(timeVector,hydPressureH(indexVector)./(1E6),lineStyle)
    ylabel('$P_H\hspace{2mm}(MPa)$','interpreter','latex','fontname','calibri','fontsize',font) 
    ylim([0 20])
    hold on
subplot1(8)
    plot(timeVector,hydPressureO(indexVector)./(1E6),lineStyle)
    ylabel('$P_O\hspace{2mm}(MPa)$','interpreter','latex','fontname','calibri','fontsize',font)  
    set(gca,'yaxislocation','right');
    ylim([0 20])
    hold on
subplot1(9)
    plot(timeVector,slipLeft(indexVector),lineStyle)
    ylim([0 100])
    ylabel('$i_L,\hspace{1mm}i_R$','interpreter','latex','fontname','calibri','fontsize',font)
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontname','calibri','fontsize',font)
    %xlabel('time (seconds)','fontsize',font)
    hold on
subplot1(10)
    plot(timeVector,engTorq2AgPumpNM(indexVector),lineStyle)
    ylabel('${\tau_{load}}\hspace{2mm}(Nm)$','interpreter','latex','fontsize',font)
    set(gca,'yaxislocation','right');
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontname','calibri','fontsize',font)
    %xlabel('time (seconds)','fontsize',font)
    hold on
  

% Thrust Slip Curves
PHI = nConstantTerrain.frictionAngle;
C = nConstantTerrain.cohesion;
K = nConstantTerrain.K;
    slip = 0:0.01:100;
    Fmax1 = (trackAreaM2*C(1) + (normalForceTrackN/1000)*tand(PHI(1)));
    F1 = 2*Fmax1*(1 - ((K(1)./(slip.*l)).*(1 - exp((-slip.*l)./K(1)))));
    Fmax2 = (trackAreaM2*C(2) + (normalForceTrackN/1000)*tand(PHI(2)));
    F2 = 2*Fmax2*(1 - ((K(2)./(slip.*l)).*(1 - exp((-slip.*l)./K(2)))));
    Fmax3 = (trackAreaM2*C(3) + (normalForceTrackN/1000)*tand(PHI(3)));
    F3 = 2*Fmax3*(1 - ((K(3)./(slip.*l)).*(1 - exp((-slip.*l)./K(3)))));


figure(205)
if plotContour
subplot1(1,2,'Gap',[0.02 0.018],'YTickL','All','Min',[0.05 0.07],'Max',[0.95 0.99],'FontS',14)
end
if plotContour
subplot1(1)
contourf(contourX,contourY,frictionAngle,'edgecolor','none')
caxis([0 30])
colormap gray
dim1 = [0.08 0.37 0.28 0.57];
str1 = 'Terrain 1';
annotation('textbox',dim1,'String',str1,'FitBoxToText','on','linestyle','none','fontsize',16,'fontAngle','italic');
dim2 = [0.2 0.37 0.4 0.57];
str2 = 'Terrain 2';
annotation('textbox',dim2,'String',str2,'FitBoxToText','on','linestyle','none','fontsize',16,'fontAngle','italic');
dim3 = [0.35 0.37 0.55 0.57];
str3 = 'Terrain 3';
annotation('textbox',dim3,'String',str3,'FitBoxToText','on','linestyle','none','fontsize',16,'fontAngle','italic');
end
hold on
subplot1(1)
plot(X,Y,lineStyle)
hold on
subplot1(1)
plot(X,Y,lineStyle)
hold on

for i=1:incVehicle:nTimeStep
  
  cosTheta = cos(theta(i));
  sinTheta = sin(theta(i));
  p1X = X(i) + l/2*cosTheta - b*sinTheta/2;
  p2X = X(i) + l/2*cosTheta + b*sinTheta/2;
  p3X = X(i) - l/2*cosTheta + b*sinTheta/2;
  p4X = X(i) - l/2*cosTheta - b*sinTheta/2;
  p1Y = Y(i) + l/2*sinTheta + b*cosTheta/2;
  p2Y = Y(i) + l/2*sinTheta - b*cosTheta/2;
  p3Y = Y(i) - l/2*sinTheta - b*cosTheta/2;
  p4Y = Y(i) - l/2*sinTheta + b*cosTheta/2;
  plot([p1X p2X p3X p4X p1X],[p1Y p2Y p3Y p4Y p1Y],lineStyle);
  plot(X(i),Y(i),lineStyle);   % cg
  hold on
  
end;
%title('Friction Angle of Terrain')
xlabel('$X_T\hspace{2mm}(m)$','interpreter','latex','fontname','calibri','fontsize',font)
xlim([0 70])
ylim([0 70])
hold on

subplot1(2)
    plot(slip,F1,'k-',slip,F2,'k--',slip,F3,'k-.')
    ylim([0 225])
    ylabel('$F_L+F_R\hspace{2mm}(kN)$','interpreter','latex','fontname','calibri','fontsize',font)
    xlabel('$i\hspace{2mm}(\%)$','interpreter','latex','fontname','calibri','fontsize',font)
    h_legend = legend(['Terrain 1: K = ',num2str(K(1)), ', \phi = ',num2str(PHI(1)), ', c = ',num2str(C(1))],['Terrain 2: K = ',num2str(K(2)), ', \phi = ',num2str(PHI(2)), ', c = ',num2str(C(2))],...
        ['Terrain 3: K = ',num2str(K(3)), ', \phi = ' ,num2str(PHI(3)), ', c = ',num2str(C(3))],'location','southeast');
    set(h_legend,'FontSize',16,'FontAngle','italic');
    set(gca,'yaxislocation','right');
    hold on
    
    
% Plots for Thesis Proposal Presentation    
AX = [0.02 0.03];
font = 16;
figure(105)
if plotContour
subplot1(2,2,'Gap',AX,'YTickL','All','Min',[0.05 0.07],'Max',[0.95 0.99],'FontS',font)
end
subplot1(1)
    plot(timeVector,vx(indexVector),lineStyle)
    ylabel('$v_T\hspace{2mm}(m/s)$','interpreter','latex','fontname','calibri','fontsize',font) 
    hold on
subplot1(2)
    plot(timeVector,vXsled(indexVector),lineStyle)
    ylabel('$v_S\hspace{2mm}(m/s)$','interpreter','latex','fontname','calibri','fontsize',font)
    set(gca,'yaxislocation','right');
    ylim([0 4])
    hold on
subplot1(3)
    plot(timeVector,DBP(indexVector)./1000,lineStyle)
    ylabel('$T\hspace{2mm}(kN)$','interpreter','latex','fontname','calibri','fontsize',font) 
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontname','calibri','fontsize',font)
    ylim([0 200])
    hold on
subplot1(4)
    plot(timeVector,slipLeft(indexVector),lineStyle)
    ylim([0 100])
    ylabel('$i_L,\hspace{1mm}i_R$','interpreter','latex','fontname','calibri','fontsize',font)
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontname','calibri','fontsize',font)
    set(gca,'yaxislocation','right');
    %xlabel('time (seconds)','fontsize',font)
    hold on

end