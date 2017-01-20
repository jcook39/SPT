function plot_follower_speed_heading_control(controller,timeStepS,nTimeStep)

% Create Time Vectors for plotting
indexVector = 1:nTimeStep;
timeVector = (indexVector-1)*timeStepS;

headingRef = controller.headingRef;
headingError = controller.headingError;
headingIntegratedError = controller.headingIntegratedError;
steerPumpCmd = controller.steerPumpCmd;

speedRef = controller.speedRef;
speedError = controller.speedError;
speedIntegratedError = controller.speedIntegratedError;
throttleCommand = controller.throttleCommand;
errorx = controller.errorx;

figure(12)
subplot(331)
    plot(timeVector,headingRef*(180/pi))
    ylabel('headingRef (deg)')
subplot(332)
    plot(timeVector,headingError*(180/pi))
    ylabel(' heading error (deg)')
subplot(333)
    plot(timeVector,headingIntegratedError*(180/pi))
    ylabel(' heading integrated error (deg) ')
subplot(334)
    plot(timeVector,steerPumpCmd)
    ylabel(' steerPumpCmd ')
subplot(335)
    plot(timeVector,speedRef)
    ylabel('speedRef (m/s)')
subplot(336)
    plot(timeVector,speedError)
    ylabel('speedError')
subplot(337)
    plot(timeVector,speedIntegratedError)
    ylabel('speedIntegratedError')
subplot(338)
    plot(timeVector,throttleCommand(indexVector))
    ylabel('throttleCommand')
subplot(339)    
    plot(timeVector,errorx)
    ylabel('errorx')

end