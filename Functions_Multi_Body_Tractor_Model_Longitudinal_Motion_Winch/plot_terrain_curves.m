function plot_terrain_curves(structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, tractorNo, figureNo, figureNoAll, tractorColor)

% --------------------- Color Declarations --------------------------------
trueColor = 'b.';
estimateColor = 'r.';

% ---------------------- Unpack Structures --------------------------------
terrainIndex = nConstantTerrain.indTrac(:,tractorNo);
terrainActual = nConstantTerrain.terrainActual(:,terrainIndex);
terrainSoft = nConstantTerrain.terrainActual(:,terrainIndex(2,1));
rollingRadiusM = nConstantMT865.rollingRadiusM;


% ------------------ Declare Dimensional Parameters -----------------------
meter2inch = 39.3701;
inch2meter = 0.0254;
grouserLengthIn = 5.5;
frontSprocketRadiusM = 0.602;
maxSinkageM = frontSprocketRadiusM + grouserLengthIn*inch2meter;
maxSinkageIn = maxSinkageM*meter2inch;
maxSinkageRangeM = 1.2;


% ===================== Unpack DTKF Struct ================================
%
% --------------------- Estimated State Vector ----------------------------
% xHat = [vHat omegaHat F_T,NetHat F_T,NetHatDot tau_ResHat tau_ResHatDot 
%           R_SHat R_SHatDot].';
%
% --------------------- True State Vector ----------------------------
% x = [v omega F_T,Net F_T,NetDot tau_Res tau_ResDot R_S R_SDot].';
%
% -------------------- Measurement Vector ---------------------------------
% y = [vDot v omega R_S].'


% ---------------- Create time Vector for plotting ------------------------
nTimeStep = nTimeParam.nTimeStep;
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;

indexVector = 1:numel(time);
timeVector = time;

% ----------------------- Unpack DTKF structure ---------------------------
x = structDTKF.x;
xHatPlus = structDTKF.xHatPlus;
xError = structDTKF.xError;
y = structDTKF.y;

slipVectorEstimate = 100*(1-(xHatPlus(1,indexVector)./(rollingRadiusM*xHatPlus(2,indexVector))));
slipVectorTrue = 100*(1-(x(1,indexVector)./(rollingRadiusM*x(2,indexVector))));
slipVectorMeasure = 100*(1-(y(2,indexVector)./(rollingRadiusM*y(3,indexVector))));

slip = 0.01:1.5:100;


[F_TNetMatActSoft,FActSoft, RActSoft, tauResActSoft, sinkageActSoft] = net_track_force(terrainSoft,nConstantMT865,slip);
[~, ~, netTractionLoadTrueSoft, ~] = peak_traction(nConstantMT865, terrainSoft, slip, 'AllTraction');

figure(figureNoAll)
    h = plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadTrueSoft./1000,tractorColor);
    set(gca,'fontname','Times New Roman','fontsize',16)
    set(h(1),'LineWidth',2)
    set(h(2),'LineWidth',2)
    ylabel('net traction with payload (kN)','interpreter','latex','fontsize',16)
    xlabel('slip ratio $i$ (\%)','interpreter','latex','fontsize',16)  
    hold on


nActualTerrain = size(terrainActual,2);
for i = 1:nActualTerrain
    [F_TNetMatAct(:,i),FAct(:,i), RAct(:,i), tauResAct(:,i), sinkageAct(:,i)] = net_track_force(terrainActual(:,i),nConstantMT865,slip);
    [~, ~, netTractionLoadMatTrue(:,i), ~] = peak_traction(nConstantMT865, terrainActual(:,i), slip, 'AllTraction');
end % end for

    figure(figureNo+3)
    set(gcf,'numbertitle','off','name',['Terrian Tractor',num2str(tractorNo)])
    subplot(231)
        plot( slip, zeros(numel(slip),1),'k:',slip, F_TNetMatAct,...
            slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
        ylabel('Net Traction (N)')
        xlabel('slip ratio')
        xlim([0 100])
        legend('Zero Mark',['Terrain',num2str(terrainIndex(1))],['Terrain',num2str(terrainIndex(2))],['Terrain',num2str(terrainIndex(3))],'Estimate','True','Location','SouthEast')
        hold on
    subplot(232)
        plot( slip, FAct)
        ylabel('Gross Traction (N)')
        xlabel('slip ratio')
        hold on
    subplot(233)
        plot(slip, tauResAct,slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
        ylabel('Resistance Torque (N)')
        xlabel('slip ratio')
        xlim([0 100])
        hold on
    subplot(234)
        plot( slip, RAct )
        ylabel('Resistance (N)')
        xlabel('slip ratio')
        hold on
    subplot(235)
        plot(slip, sinkageAct,...
            slip, maxSinkageIn*inch2meter*ones(numel(slip),1),'k:')
        xlabel('slip ratio')
        ylabel('sinkage (m)')
        hold on
    subplot(236)
        plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadMatTrue)
        ylabel('Net Traction with PayLoad (N)')
        xlabel('slip ratio')        

        
        
        
% 
%     fontLabel = 16;
%     nActualTerrain = size(terrainActual,2);
%     for i = 1:nActualTerrain
%         [F_TNetMatAct(:,i),FAct(:,i), RAct(:,i), tauResAct(:,i), sinkageAct(:,i)] = net_track_force(terrainActual(:,i),nConstantMT865,slip);
%         [~, ~, netTractionLoadMatTrue(:,i), ~] = peak_traction(nConstantMT865, terrainActual(:,i), slip, 'AllTraction');
%     end % end for
% 
%     figure(figureNo+4)
%     subplot(131)
%         plot( slip, F_TNetMatAct(:,1), slip, F_TNetMatAct(:,2),...
%             slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
%         ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         xlim([-10 100])
%         ylim([0 15E4])
%         legend('Terrain 1','Terrain 2','Estimate','True','Location','SouthEast')
%         hold on
% 
%     subplot(132)
%         plot(slip, tauResAct(:,1), slip, tauResAct(:,2),...
%             slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
%         ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         xlim([-10 100])
%         ylim([0 15E4])
%         legend('Terrain 1','Terrain 2','Estimated','True','Location','SouthEast')
%         hold on
% 
%     subplot(133)
%         plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadMatTrue(:,1), slip, netTractionLoadMatTrue(:,2))
%         ylabel('Net Traction with Payload (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
% 
% 
%     figure(figureNo+5)
%     subplot(121)
%         plot( slip, F_TNetMatAct(:,1), slip, F_TNetMatAct(:,2),...
%             slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
%         ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         xlim([-10 100])
%         ylim([0 15E4])
%         legend('Terrain 1','Terrain 2','Estimate','True','Location','SouthEast')
%         hold on
% 
%     subplot(122)
%         plot(slip, tauResAct(:,1), slip, tauResAct(:,2),...
%             slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
%         ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         xlim([-10 100])
%         ylim([0 15E4])
%         legend('Terrain 1','Terrain 2','Estimated','True','Location','SouthEast')
%         hold on
% 
% 
%     figure(figureNo+6)
%     plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadMatTrue(:,1), slip, netTractionLoadMatTrue(:,2))
%         ylabel('Net Traction with Payload (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         legend('zero mark','Terrain 1','Terrain 2')
%     

end
