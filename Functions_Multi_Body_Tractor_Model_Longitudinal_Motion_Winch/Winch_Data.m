% This script looks at Winch Data for the following product:
% http://www.team-twg.com/documents/dp-winch/sales-literature/model-45-technical-drawing.pdf
% Use Motor Option 2

linePullLBS = [45000 38100 33000 29100 26100 23600].'
lineSpdFPM = [18 21 25 28 31 34].'

% Unit Conversion
linePullN = linePullLBS*4.44822;
lineSpdMPS = lineSpdFPM*0.00508;

% Flow
q = 25*6.30902e-5;

%Properties
radiusWinchM = 0.1905;
omegaWinchRadPS = lineSpdMPS/radiusWinchM;
Dest1 = q./omegaWinchRadPS;
figure(2)
plot(Dest1)

figure(1)
plot(lineSpdMPS,linePullN,'r*')