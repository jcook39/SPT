function simulation_video( nTractor, nTractorColor, nConstantMT865, nConstantTerrain, waypoint, timeStepS, nTimeStep, videoType, plotTerrain, filename)
% This function creates a video to play back simulation results

%%%%%%%%%%%%%%%%%% Evaluate Tractor Structure Size %%%%%%%%%%%%%%%%%%%%%%%%
nTractorName = fieldnames(nTractor);
nTractorColorName = fieldnames(nTractorColor);
nTractorNum = size(nTractorName,1); % Determine number of tractors

%%%%%%%%%%%%%%%%%%%%%%%%% Unpack Terrain %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
contourX = nConstantTerrain.X;
contourY = nConstantTerrain.Y;
frictionAngle = nConstantTerrain.frictionAngle;
cohesion = nConstantTerrain.cohesion;

%%%%%%%%%%%%%%%%%%%%%%%%%%% FRAME RATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
videoPlayRate = 1; % Example: 2 means twice as fast
timePerFrame = (1/videoPlayRate)*timeStepS; %
nTimeStepPerFrame = 1;

videoTimeArrayIndex = 1:nTimeStepPerFrame:nTimeStep;
videoTimeArray = (videoTimeArrayIndex.*timeStepS) - timeStepS;
videoTimeArraySize = numel(videoTimeArray);

%%%%%%%%%%%%%%%%%%%%%%%% UNPACK WAY POINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nWayPointFollowEvalMatTime = waypoint.nWayPointFollowEvalMatTime;
nWayPointAllTime = waypoint.nWayPointAllTime;


%%%%%%%%%%%%%%%%%%%%%%%%% Simulation Video %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(videoType,'video')
    video = VideoWriter(filename);
    %video.FrameRate = 1/timeStepS;
    video.FrameRate = 1/timePerFrame;
    video.Quality = 20;
    open(video);
end
lineWidthSize = 2;


%%%%%%%%%%%%%%%%%%%%%%%% Declare Arrays %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X = zeros(nTractorNum,videoTimeArraySize);
Y = zeros(nTractorNum,videoTimeArraySize);
theta = zeros(nTractorNum,videoTimeArraySize);
phi = zeros(nTractorNum,videoTimeArraySize);
si = zeros(nTractorNum,videoTimeArraySize);
vx = zeros(nTractorNum,videoTimeArraySize);
vy = zeros(nTractorNum,videoTimeArraySize);
dtheta = zeros(nTractorNum,videoTimeArraySize);
dphi = zeros(nTractorNum,videoTimeArraySize);
dsi = zeros(nTractorNum,videoTimeArraySize);
wl = zeros(nTractorNum,videoTimeArraySize);
wr = zeros(nTractorNum,videoTimeArraySize);

engSpeedRadPS = zeros(nTractorNum,videoTimeArraySize);

speed = zeros(nTractorNum,videoTimeArraySize);
slipLeft = zeros(nTractorNum,videoTimeArraySize);
slipRight = zeros(nTractorNum,videoTimeArraySize);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:nTractorNum
    tractor = nTractor.(nTractorName{j});
            
    for i = 1:nTimeStepPerFrame:nTimeStep

    % Gather Tractor States
    X(j,i) = tractor(i).state(1);
    Y(j,i) = tractor(i).state(2);
    theta(j,i) = tractor(i).state(3);
    phi(j,i) = tractor(i).state(4);
    si(j,i) = tractor(i).state(5);
    vx(j,i) = tractor(i).state(6);
    vy(j,i) = tractor(i).state(7);
    dtheta(j,i) = tractor(i).state(8);
    dphi(j,i) = tractor(i).state(9);
    dsi(j,i) = tractor(i).state(10);
    wl(j,i) = tractor(i).state(11);
    wr(j,i) = tractor(i).state(12);

    engSpeedRadPS(j,i) = tractor(i).state(14);

    % Augmented States
    speed(j,i) = sqrt( vx(j,i)^2 + vy(j,i)^2 );       
    slipLeft(j,i) = tractor(i).slip(1);
    slipRight(j,i) = tractor(i).slip(2);
    
    end
    
end

if strcmp(plotTerrain,'yes')
    wayPointColor = 'wo';
    selectedWayPointColor = 'bx';
else
    wayPointColor = 'ko'
    selectedWayPointColor = 'wx';
end

%%%%%%%%%%%%%%%%%%%%%%%% Figure Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fontLabel = 16;
FigHandle = figure;
set(FigHandle, 'Position', [0, 0, 1920, 1200]);
    for i = 1:nTimeStepPerFrame:nTimeStep
        if strcmp(plotTerrain,'yes')            
            subplot(221)
            contourf(contourX,contourY,frictionAngle,'edgecolor','none')
            colormap winter
            hold on
        end
        
        subplot(221)
        cla
        
        for j = 1:nTractorNum
            % Plot Each Tractor
            tractor = nTractor.(nTractorName{j});
            tractorColor = nTractorColor.(nTractorColorName{j});
            
            % Plots
            subplot(221)
                %plot(nWayPointAllTime(1:i,1), nWayPointAllTime(1:i,2),'wo','linewidth',4)
                %hold on
                %plot(nWayPointFollowEvalMatTime(i,1),nWayPointFollowEvalMatTime(i,2),'bx','linewidth',2)
                %hold on
                plot_tractor_on_map(tractor,i,nConstantMT865,tractorColor)
                %title('Friction Angle of Terrain')
                ylabel('East Position (meters)','interpreter','latex','FontSize',fontLabel)
                xlabel('North Position (meters)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                title(['t = ',num2str((i-1)*timeStepS)])
                hold on
                plot( X(j,1:i), Y(j,1:i), tractorColor, 'linewidth', 2)
                hold on
            
            subplot(422)
                h = plot(videoTimeArray(i),speed(j,i),tractorColor);
                set(h(1),'linewidth',lineWidthSize)
                ylabel('Vehicle Speed (m/s)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                xlim([0 videoTimeArray(end)])
                ylim([0 4])
                hold on
                
            subplot(424)
                h = plot(videoTimeArray(i),dtheta(j,i),tractorColor);
                set(h(1),'linewidth',lineWidthSize)
                ylabel('Yaw Rate (rad/s)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                xlim([0 videoTimeArray(end)])
                ylim([-0.25 0.25])
                hold on
                
            subplot(426)
                h = plot(videoTimeArray(i),slipLeft(j,i),tractorColor);
                set(h(1),'linewidth',lineWidthSize)
                ylabel('Slip Ratio Left','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                xlim([0 videoTimeArray(end)])
                ylim([0 10])
                hold on
                
            subplot(428)
                h = plot(videoTimeArray(i),slipRight(j,i),tractorColor);
                set(h(1),'linewidth',lineWidthSize)
                ylabel('Slip Ratio Right','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                xlim([0 videoTimeArray(end)])
                ylim([0 10])
                hold on
                
            subplot(425)
                h = plot(videoTimeArray(i),engSpeedRadPS(j,i).*(60/(2*pi)),tractorColor);
                set(h(1),'linewidth',lineWidthSize)
                ylabel('Engine Speed (RPM)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                xlim([0 videoTimeArray(end)])
                ylim([1000 2300])
                hold on
                
            subplot(427)
                h = plot(videoTimeArray(i),phi(j,i).*(180/pi),tractorColor);
                set(h(1),'linewidth',lineWidthSize)
                ylabel('DrawBar Angle (Degrees)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
                xlim([0 videoTimeArray(end)])
                ylim([-15 15])
                hold on
     
        end % END FOR nTractorNum
        
        
        % Compute way points to plot
        thetaError = atan2( nWayPointAllTime(1:i,2) - Y(2,i) , nWayPointAllTime(1:i,1) - X(2,i) ) - theta(2,i);
        nWayPointX = nWayPointAllTime( abs(thetaError) < (pi/2) , 1);
        nWayPointY = nWayPointAllTime( abs(thetaError) < (pi/2) , 2);
        
        subplot(221)
            plot(nWayPointX, nWayPointY,wayPointColor,'linewidth',4)
            hold on
            plot(nWayPointFollowEvalMatTime(i,1),nWayPointFollowEvalMatTime(i,2),selectedWayPointColor,'linewidth',2)
            hold on
            xlim([min(X(2,i))-50 max(X(2,i))+75])
            ylim([min(Y(2,i))-50 max(Y(2,i))+50])
            set(gca,'FontSize',fontLabel,'FontName','Times New Roman')
            hold on
            % hold off
            
        if strcmp(videoType,'video')
            movieVar(i) = getframe(gcf);
            writeVideo(video,movieVar(i));
        elseif strcmp(videoType,'matlab')
            pause(0.01)
        end
        %hold on
        % hold off
    end % END FOR Figure Loop for each time step
    
    if strcmp(videoType,'video')
        close(video);
    end
    
    
end

