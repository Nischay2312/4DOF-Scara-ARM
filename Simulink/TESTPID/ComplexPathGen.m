%Script to Generate Path
%all measurements are in Centimeters
totaltime = 2.2;
%Rectangle
InputX = [38.97 26 10 10 26 26]';
InputY = [ 22.5 15 15 25 25 15]';

%Letter N
%InputX = [38.97 10 10 26 26 38.97]';
%InputY = [22.50 15 35 15 35 22.50]';

%InputX = [38.97 38.97]';
%InputY = [22.50 22.50]';

numlines = length(InputX)-1;
timeNeeded =totaltime/numlines;   %this will dictate the speed of the path.
timeSteps = timeNeeded ./ SimStepTime;
slope = zeros((numlines), 1);
const = slope;
Xstep = slope;
Ystep = slope;
%Calculate slopes and constants and steps
%straight line of the from y = mx + c
for i = 1 : (numlines)
    slope(i) = (InputY(i+1) - InputY(i))/(InputX(i+1) - InputX(i));
    const(i) = InputY(i) - slope(i) * InputX(i);
    Xstep(i) = (InputX(i+1) - InputX(i))/timeSteps;
    Ystep(i) = (InputY(i+1) - InputY(i))/timeSteps;
end

%Variables to store the position data
Xpos = zeros(length(SimTimeVector), 1);
Ypos = Xpos;
%initialize them
Xpos(1) = InputX(1);
Ypos(1) = InputY(1);

currentpos = 2;

%Generate the path
%generate lines sequentially
for j = 1: (numlines)
    for i = currentpos: length(SimTimeVector)
        if(SimTimeVector(i) <= (timeNeeded * j))
            if(Xstep(j) == 0)
                Ypos(i) = Ypos(i-1) + Ystep(j);
                Xpos(i) = Xpos(i-1);
            else
                Xpos(i) = Xpos(i - 1) + Xstep(j);
                Ypos(i) = Xpos(i) * slope(j) + const(j);
            end
        elseif(j == numlines)
            Xpos(i) = Xpos(i - 1);
            Ypos(i) = Ypos(i - 1);
        else
            currentpos = i;
            break;
        end
    end
end

%finally Convert output to suitable datatype ~~Timeseries
XposT = timeseries(Xpos, SimTimeVector);
YposT = timeseries(Ypos, SimTimeVector);
figure(2)
plot(Xpos, Ypos)
xlim([-50 50]);
ylim([-50 50]);