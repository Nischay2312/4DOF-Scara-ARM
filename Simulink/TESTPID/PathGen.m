%Script to Generate Path
%all measurements are in Centimeters
timeNeeded = 2.0;   %this will dictate the speed of the path.
timeSteps = timeNeeded ./ SimStepTime;
startX = 45.0;
startY = 0.0;
endX = 20.0;
endY = 00.0;

%now we generate a straight line of the from y = mx + c
slope = (endY - startY)/(endX - startX);
%now get c
const = endY - slope * endX;

%now we make the path 
Xstep = (endX - startX)/timeSteps;
Xpos = zeros(length(SimTimeVector), 1);
Ypos = Xpos;
Xpos(1) = startX;
Ypos(1) = startY;

%Generate the path
for i = 2: length(SimTimeVector)
   if((SimTimeVector(i) <= timeNeeded))  
       Xpos(i) = Xpos(i - 1) + Xstep;
       Ypos(i) = Xpos(i) * slope + const;
   else
       Xpos(i) = Xpos(i - 1);
       Ypos(i) = Ypos(i - 1);
   end
end

%finally Convert output to suitable datatype ~~Timeseries
XposT = timeseries(Xpos, SimTimeVector);
YposT = timeseries(Ypos, SimTimeVector);
figure(2)
plot(Xpos, Ypos)