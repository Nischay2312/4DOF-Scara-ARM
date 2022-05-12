%Script to save X and Y Coordinates
%Saving Coord Output
BaseL = 30.0;
ArmL = 15.0;

posX = out.Xcords;
posY = out.Ycords;

Xc = posX.data;
Yc = posY.data;
time = posX.time;

for i = 1: length(time)
    [BaseAngle(i), ArmAngle(i)] = InverseKin(Xc(i), Yc(i));
end
BaseAngle = BaseAngle';
ArmAngle = ArmAngle';

h = animatedline('MaximumNumPoints',5);
figure(3)
hold on

a = tic;
for i = 1 :25: length(time)
    ylim([-50 50]);
    xlim([-50 50]);
    arm1X = BaseL * cos(BaseAngle(i)*pi/180);
    arm1Y = BaseL * sin(BaseAngle(i)*pi/180);
    arm1Xx = [0 arm1X];
    arm1Yy = [0 arm1Y];

    arm2X = ArmL * cos((ArmAngle(i)+BaseAngle(i))*pi/180);
    arm2Y = ArmL * sin((ArmAngle(i)+BaseAngle(i))*pi/180);
    arm2Xx = [arm1X, arm2X];
    arm2Yy = [arm1Y, arm2Y];
    
    
    h1 = plot(arm1Xx, arm1Yy, '-b', 'LineWidth', 3);
    h2 = plot(arm2Xx, arm2Yy, '-g', 'LineWidth', 3);
    h3 = plot(Xc(i), Yc(i), 'LineWidth', 3);
    
    %addpoints(h,Xc(i), Yc(i));
    %addpoints(h, arm1Xx, arm1Yy);
    %addpoints(h, arm2Xx, arm2Yy);
    b = toc(a); % check timer
    if b > (1/5000000000)
        drawnow % update screen every 1/30 seconds
        a = tic; % reset timer after updating
    end
    if(i ~= (length(time) - 1))
        delete(h1);
        delete(h2);
        delete(h3);
    end
    %drawnow 
end
    h1 = plot(arm1Xx, arm1Yy, '-b', 'LineWidth', 3);
    h2 = plot(arm2Xx, arm2Yy, '-g', 'LineWidth', 3);
    h3 = plot(Xc(i), Yc(i), 'LineWidth', 3);
drawnow 





