function [ engineTorqueNM ] = engine_interp( throttle, engineControlThrottle, engineSpeedRadSec, nConstantMT865)
%This function calculates the torque coming out of the engine as a function
%of the normalized throttle input, engine speed, and controller normalized
%throttle input.

engineDataRadPS = nConstantMT865.engineSpeedDataRadPS; 
engineTorqueDataNM = nConstantMT865.engineTorqueDataNM;
MaxEngSpdRadPS = 2100*((2*pi)/60);

throttleActual = max(throttle,engineControlThrottle);

if throttleActual > 1
    throttleActual = 1;
elseif throttleActual < 0
    throttleActual = 0;
end

engineTorqueMax = interp1(engineDataRadPS,engineTorqueDataNM,engineSpeedRadSec,'spline');
engineTorqueNM = throttleActual.*engineTorqueMax.*(engineSpeedRadSec < MaxEngSpdRadPS);

end