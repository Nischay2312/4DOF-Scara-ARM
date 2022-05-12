function [x, y] = ForwardKin(t1, t2)
    
    Larm1 = 35;
    Larm2 = 15;
    
    arm1X = Larm1 * cos(t1*pi/180);
    arm1Y = Larm1 * sin(t1*pi/180);

    arm2X = Larm2 * cos(t2*pi/180) + arm1X;
    arm2Y = Larm2 * sin(t2*pi/180) + arm1Y;
    
    y = arm2Y;
    x = arm2X;
end