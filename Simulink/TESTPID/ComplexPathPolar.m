%Script to Generate Path POLAR Cords
%all measurements are in Centimeters
totaltime = 2.0;

InputR = [45 45 45 45 45 45 45 45 45 45 45 45 45 45  45  45  45  45  45  45  45  45  45  45  45  40  35  30  25  20  20  20  20  20  20  20  20  20  20  20  20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 25 30 35 40 45];
InputT = [30 35 40 45 50 55 60 65 70 75 80 85 90 95 100 105 110 115 120 125 130 135 140 145 150 150 150 150 150 150 150 145 140 135 130 125 120 115 110 105 100 95 90 85 80 75 70 65 60 55 50 45 40 35 30 30 30 30 30 30 30];

%InputR = [40 40 25 20  15 15 35 20 20  20  40  20 45 20 35 40];
%InputT = [30 30 70 90 110 70 85 30 90 150 110 105 90 70 85 30];


InputT = InputT * pi/180;
PositionVectorLength = length(InputR);
InputX = zeros(PositionVectorLength, 1);
InputY = InputX;

for i = 1 : PositionVectorLength
    InputX(i) = InputR(i) * cos(InputT(i));
    InputY(i) = InputR(i) * sin(InputT(i));
end

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
figure(3)
plot(Xpos, Ypos)
xlim([-50 50]);
ylim([-50 50]);