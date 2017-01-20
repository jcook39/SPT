function movieVar = plot_result(tractor,inputMat,lineStyle,nTimeStep,nConstantTerrain,nConstantMT865,timeStepS,tractorColor,plotContour)


%% Create time Vector for plotting
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


for i = 1:nTimeStep
        % State Vector
        X(i) = tractor(i).state(1);
        Y(i) = tractor(i).state(2);
        theta(i) = tractor(i).state(3);
        phi(i) = tractor(i).state(4);
        si(i) = tractor(i).state(5);
        vx(i) = tractor(i).state(6);
        vy(i) = tractor(i).state(7);
        thetaDot(i) = tractor(i).state(8);
        phiDot(i) = tractor(i).state(9);
        siDot(i) = tractor(i).state(10);
        wl(i) = tractor(i).state(11);
        wr(i) = tractor(i).state(12);
        engThrtlState(i) = tractor(i).state(13);
        engSpdRadPS(i) = tractor(i).state(14);
        clutchComState(i) = tractor(i).state(15);
        engCtrlThrtl(i) = tractor(i).state(16);
        steerAngleState(i) = tractor(i).state(17);
        pressureHPa(i) = tractor(i).state(18);
        % Other Data
        clutchIsSlip(i) = tractor(i).clutchIsSlip;
        slipLeft(i) = tractor(i).slip(1);
        slipRight(i) = tractor(i).slip(2);
        TR(i) = tractor(i).torqueRightSprocketNM;
        TL(i) = tractor(i).torqueLeftSprocketNM;

        forces(i,:) = tractor(i).forces;

        %xdot
        vxd(i) = tractor(i).xdot(6);
        vyd(i) = tractor(i).xdot(7);
        thetaDDot(i) = tractor(i).xdot(8);

        % Sled Forces
        Sledx3(i) = tractor(i).sledForcexy(1);
        Sledy3(i) = tractor(i).sledForcexy(2);
        vSledx(i) = tractor(i).vSledxy(1);
        vSledy(i) = tractor(i).vSledxy(2);
        vSledz(i) = tractor(i).vSledxy(3);
        
        enginePowerW(i) = tractor(i).enginePowerW(1);
        
        steerMotorTorqueNM(i) = tractor(i).steerMotorTorqueNM;
        

end

%% Unpack Constant Parameters
    b = nConstantMT865.trackCG2trackCG;
    l = nConstantMT865.trackLengthM;
    w = nConstantMT865.trackWidthM;
    TCG2H = nConstantMT865.TTCG2Hitch;
    lSled = nConstantMT865.sledLengthM;
    bSled = nConstantMT865.sledWidthM;
    lArm = nConstantMT865.towArmM;
    lTool = nConstantMT865.CRLTool;

%% Unpack Terrain Plat
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
contourX = nConstantTerrain.X;
contourY = nConstantTerrain.Y;
frictionAngle = nConstantTerrain.frictionAngle;
cohesion = nConstantTerrain.cohesion;

%% Spatial Plots
timeStepVehicle = 20; % seconds
incVehicle = timeStepVehicle/timeStepS;

% Plot the terrain contours if requested
figure(1)
if strcmp(plotContour,'plotContour')
    AX = [0.05 0.05];
    subplot1(1,1,'Gap',AX,'YTickL','All','XTickL','All','Min',[0.1 0.1],'Max',[0.99 0.99],'FontS',16)
    contourf(contourX,contourY,frictionAngle,'edgecolor','none')
    caxis([0 30])
    %colormap gray
    colormap bone
    %colorbar('EastOutside')
    hold on
end
lineWidthTractorTraj = 2;
plot(X,Y,lineStyle, 'linewidth', lineWidthTractorTraj )
hold on

% Plot snapshots of the tractor
% for i=1:incVehicle:nTimeStep
%     plot_tractor_on_map(tractor,i,nConstantMT865,tractorColor)
%     hold on
% end;
plot_tractor_on_map(tractor,1,nConstantMT865,tractorColor)
plot_tractor_on_map(tractor,nTimeStep,nConstantMT865,tractorColor)
% xStartArrow = [0.205 0.205];
% yStartArrow = [0.7 0.6];
% annotation('textarrow', xStartArrow, yStartArrow, 'String', 'Start Position','fontsize',16)

%title('Friction Angle of Terrain')
ylabel('North Position (meters)','interpreter','latex','FontSize',20,'fontname','timesnewroman')
xlabel('East Position (meters)','interpreter','latex','FontSize',20,'fontname','timesnewroman')
set(gca,'FontSize',20)
xlim([25 325])
ylim([150 250])
hold on

figure(2)
if strcmp(plotContour,'plotContour')
    contourf(contourX,contourY,cohesion,'edgecolor','none')
    caxis([0 30])
    colormap gray
    colorbar('EastOutside')
    hold on
end

plot(X,Y,lineStyle);
hold on

for i=1:incVehicle:nTimeStep
    plot_tractor_on_map(tractor,i,nConstantMT865,tractorColor)
    hold on
end;
title('Cohesion of Terrain')
ylabel('lateral position (m)')
xlabel('longitudinal position (m)')
xlim([0 200])
ylim([100 300])
hold on

%% Plots
figure(3)
subplot(421)
    plot(timeVector,X(indexVector),tractorColor)
    ylabel('X Pos (m)')
    hold on
subplot(422)
    plot(timeVector,Y(indexVector),tractorColor)
    ylabel('Y Pos (m)')
    hold on
subplot(423)
    plot(timeVector,theta(indexVector),tractorColor)
    ylabel('yaw Angle ')
    hold on
subplot(424)
    plot(timeVector,throttle(indexVector),timeVector,engThrtlState,tractorColor,timeVector,engCtrlThrtl(indexVector))
    legend('Throttle Cmd','Throttle Cmd State','Eng Ctrl Thrtl')
    ylim([0 1])
    hold on
subplot(425)
    plot(timeVector,gear(indexVector),tractorColor)
    ylabel('gear Selection')
    hold on
subplot(426)
    plot(timeVector,steerAngleDeg(indexVector),tractorColor)
    ylabel('steer Angle Deg')
    hold on
subplot(427)
    plot(timeVector,clutchCmd(indexVector),timeVector,clutchComState,tractorColor)
    legend('Commanded Engagement','Actual Engagement')
    ylim([0 1])
    hold on
subplot(428)
    plot(timeVector,clutchIsSlip(indexVector),tractorColor)
    ylabel('Clutch is Slipping = 1')
    
figure(4)
subplot(421)
    plot(timeVector,vx(indexVector),tractorColor)
    ylabel('vx (m/s)')   
    hold on
subplot(422)
    plot(timeVector,vy(indexVector),tractorColor)
    ylabel('vy (m/s)')   
    hold on
subplot(423)
    plot(timeVector,thetaDot(indexVector),tractorColor)
    ylabel('thetaDot (rad/s)')  
    hold on
subplot(424)
    plot(timeVector,wl(indexVector),tractorColor)
    ylabel('wl (rad/s)')   
    hold on
subplot(425)
    plot(timeVector,wr(indexVector),tractorColor)
    ylabel('wr (rad/s)')   
    hold on
subplot(426)
    plot(timeVector,engThrtlState(indexVector),tractorColor)
    ylabel('engine Thrtl State (Nm)')
    hold on
subplot(427)
    plot(timeVector,engSpdRadPS(indexVector).*((60)/(2*pi)),tractorColor)
    %ylim([1000 2300])
    ylabel('eng Speed (RPM)')
    hold on
subplot(428)
    plot(timeVector,atan2(vy,vx))
    ylabel('side slip')
    hold on
    
figure(5)
subplot(421)
    plot(timeVector,phi(indexVector),tractorColor)
    ylabel('phi')
    hold on
subplot(422)
    plot(timeVector,phiDot(indexVector),tractorColor)
    ylabel('phiDot')
    hold on
subplot(423)
    plot(timeVector,si(indexVector),tractorColor)
    ylabel('si')
    hold on
subplot(424)
    plot(timeVector,siDot(indexVector),tractorColor)
    ylabel('siDot')
    hold on
subplot(425)
    plot(timeVector,pressureHPa(indexVector)./6894.76,tractorColor)
    ylabel('Pressure H (Psi)')
    hold on
subplot(426)
    plot(timeVector,steerMotorTorqueNM(indexVector))
    ylabel('steerMotorTorqueNM')
    hold on
    
figure(6)
subplot(121)
    plot(timeVector,slipLeft(indexVector),tractorColor)
    ylim([min(slipLeft) max(slipLeft)])
    ylabel('Slip Ratio Left')
    hold on
subplot(122)
    plot(timeVector,slipRight(indexVector),tractorColor)
    ylim([min(slipRight) max(slipRight)])
    ylabel('Slip Ratio Right')  
    hold on

Fl = forces(:,1);
Fr = forces(:,2);
    
RlLF = forces(:,5);
RlRF = forces(:,6);
RlLR = forces(:,7);
RlRR = forces(:,8);
figure(7)
subplot(421)
    plot(timeVector,TL(indexVector),tractorColor)
    ylabel('Left Torque Nm')
    hold on
subplot(422)
    plot(timeVector,TR(indexVector),tractorColor)
    ylabel('Right Torque Nm')
    hold on
subplot(423)
    plot(timeVector,RlLF,tractorColor)
    ylabel('RlLF (N)')
    hold on
subplot(424)
    plot(timeVector,RlRF,tractorColor)
    ylabel('RlRF (N)')
    hold on
subplot(425)
    plot(timeVector,RlLR,tractorColor)
    ylabel('RlLR (N)')
    hold on
subplot(426)
    plot(timeVector,RlRR,tractorColor)
    ylabel('RlRR (N)')
    hold on
subplot(427)
    plot(timeVector,Fl,tractorColor)
    ylabel('Fl')
    hold on
subplot(428)   
    plot(timeVector,Fr,tractorColor)
    ylabel('Fr')
    hold on
    
figure(8)
subplot(321)
    plot(timeVector,vxd(indexVector),tractorColor)
    ylabel('vxd')
    hold on
subplot(322)
    plot(timeVector,vyd(indexVector),tractorColor)
    ylabel('vyd')
    hold on
subplot(323)
    plot(timeVector,thetaDDot(indexVector),tractorColor)
    ylabel('thetaDDot')
    hold on
    
figure(9)
subplot(211)
    plot(timeVector,Sledx3(indexVector),'r',timeVector,Sledy3(indexVector),'b',timeVector,sqrt( Sledx3(indexVector).^2  + Sledy3(indexVector).^2 ),'g' )
    legend('Sledx3','Sledy3','Magnitude')
    hold on
subplot(212)
    plot(timeVector,vSledx(indexVector),'r',timeVector,vSledy(indexVector),'b',timeVector,vSledz(indexVector),'g')
    legend('vSledx','vSledy','vSledz')
    hold on

AX = [0.05 0.05];
font = 15;
fontNum = 15;
figure(10)
if plotContour
subplot1(3,2,'Gap',AX,'YTickL','All','Min',[0.05 0.07],'Max',[0.95 0.99],'FontS',fontNum)
end
subplot1(1)
    plot(timeVector,sqrt(vx.^2 + vy.^2),lineStyle)
    %ylabel('vehicle Speed')
    ylabel('$\|v_T\|\hspace{2mm}(m/s)$','interpreter','latex','fontname','calibri','fontsize',font)
    hold on
subplot1(2)
    plot(timeVector,thetaDot,lineStyle)
    %ylabel('Yaw Rate')
    ylabel('$\dot{\theta}\hspace{2mm}(rad/s)$','interpreter','latex','fontname','calibri','fontsize',font)
    set(gca,'yaxislocation','right');
    hold on
subplot1(3)
    plot(timeVector,slipLeft(indexVector),lineStyle)
    ylim([0 100])
    ylabel('$i_L\hspace{2mm}(\%)$','interpreter','latex','fontname','calibri','fontsize',font)
    %ylabel('Slip Ratio Left')
    hold on
subplot1(4)
    plot(timeVector,slipRight(indexVector),lineStyle)
    ylim([0 100])
    %ylabel('Slip Ratio Right')  
    ylabel('$i_R\hspace{2mm}(\%)$','interpreter','latex','fontname','calibri','fontsize',font)
    set(gca,'yaxislocation','right');
    hold on
subplot1(5)
    plot(timeVector,engSpdRadPS(indexVector).*((60)/(2*pi)).*(1/1000),lineStyle)
    ylabel('$\Omega\hspace{2mm}(RPM\times1000)$','interpreter','latex','fontname','calibri','fontsize',font)
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontname','calibri','fontsize',font)
    ylim([1000/1000 2300/1000])
    hold on
subplot1(6)
    plot(timeVector,enginePowerW(indexVector)./1000,lineStyle)
    %ylabel('Engine Power (kW)')
    ylabel('$P_E\hspace{2mm}(kW)$','interpreter','latex','fontname','calibri','fontsize',font)
    set(gca,'yaxislocation','right');
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontname','calibri','fontsize',font)
    hold on


% plot(timeVector,DBP(indexVector)./1000,lineStyle)
%     ylabel('$T\hspace{2mm}(kN)$','interpreter','latex','fontname','calibri','fontsize',font) 
%     set(gca,'yaxislocation','right');
%     ylim([0 200])
%     hold on
% 

AX = [0.05 0.05];
font = 18;
fontNum = 18;
figure(12)
if plotContour
subplot1(3,2,'Gap',AX,'YTickL','All','Min',[0.1 0.1],'Max',[0.95 0.95],'FontS',fontNum)
end
subplot1(1)
    plot(timeVector,sqrt(vx.^2 + vy.^2),lineStyle,'linewidth', lineWidthTractorTraj)
    %ylabel('vehicle Speed')
    ylabel('$\|v_T\|\hspace{2mm}(m/s)$','interpreter','latex','fontsize',font)
    hold on
subplot1(2)
    plot(timeVector,thetaDot,lineStyle,'linewidth', lineWidthTractorTraj)
    %ylabel('Yaw Rate')
    ylabel('$\dot{\theta}\hspace{2mm}(rad/s)$','interpreter','latex','fontsize',font)
    set(gca,'yaxislocation','right');
    hold on
subplot1(3)
    plot(timeVector,slipLeft(indexVector),lineStyle,'linewidth', lineWidthTractorTraj)
    ylim([0 10])
    ylabel('$i_L\hspace{2mm}(\%)$','interpreter','latex','fontsize',font)
    %ylabel('Slip Ratio Left')
    hold on
subplot1(4)
    plot(timeVector,slipRight(indexVector),lineStyle,'linewidth', lineWidthTractorTraj)
    ylim([0 10])
    %ylabel('Slip Ratio Right')  
    ylabel('$i_R\hspace{2mm}(\%)$','interpreter','latex','fontsize',font)
    set(gca,'yaxislocation','right');
    hold on
subplot1(5)
    plot(timeVector,engSpdRadPS(indexVector).*((60)/(2*pi)).*(1/1000),lineStyle,'linewidth', lineWidthTractorTraj)
    ylabel('$\Omega\hspace{2mm}(RPM\times1000)$','interpreter','latex','fontsize',font)
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontsize',font)
    ylim([1300/1000 2100/1000])
    hold on
subplot1(6)
    plot(timeVector,phi(indexVector)*(180/pi),lineStyle,'linewidth', lineWidthTractorTraj)
    ylabel('$\phi\hspace{2mm}(deg)$','interpreter','latex','fontsize',font)
    set(gca,'yaxislocation','right');
    xlabel('$time\hspace{2mm}(seconds)$','interpreter','latex','fontsize',font)
    hold on

   
end
 
