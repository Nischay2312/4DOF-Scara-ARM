%R axis Data gen script
totaltime = 2;

%Rectangle
% InputX = [38.97 26 10 10 26 26]';
% InputY = [ 22.5 15 15 25 25 15]';

%Letter N
Angle = [0 360 360 0 0 -40-40]';
numlines = length(Angle)-1;
timeNeeded =totaltime/numlines;   %this will dictate the speed of the path.
timeSteps = timeNeeded ./ SimStepTime;
Astep = zeros((numlines), 1);

%Calculate the Angle Steps
for i = 1 : (numlines)
    Astep(i) = (Angle(i+1) - Angle(i))/timeSteps;
end

%Variables to store the position data
RAng = zeros(length(SimTimeVector), 1);
%initialize them
RAng(1) = Angle(1);

currentpos = 2;

%Generate the path
%generate lines sequentially
for j = 1: (numlines)
    for i = currentpos: length(SimTimeVector)
        if(SimTimeVector(i) <= (timeNeeded * j))
            RAng(i) = RAng(i-1) + Astep(j);
        elseif(j == numlines)
            RAng(i) = RAng(i - 1);
        else
            currentpos = i;
            break;
        end
    end
end

%finally Convert output to suitable datatype ~~Timeseries
RAngT = timeseries(RAng, SimTimeVector);
figure(2)
plot(RAngT)
xlim([0 SimTime]);
ylim([-360 360]);