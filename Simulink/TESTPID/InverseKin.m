function [t1, t2] = InverseKin(x, y)
    L1 = 35;
    L2 = 15;
    L3 = sqrt(x.^2 + y.^2);
    
    a = acos((L1.^2 + L3.^2 - L2.^2)/(2.*L1.*L3));
    b = acos((L1.^2 + L2.^2 - L3.^2)/(2.*L2.*L1));
    
    Theta = atan2(y,x);
    
    t1 = real((Theta + a).*180/pi);
    t2 = real(((b - (pi - t1.*pi/180))*180/pi)); 
end
