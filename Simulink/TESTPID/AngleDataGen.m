%Script to save the Angle Data of the robot.
BaseAngle = cast((out.BAngle.data)', "int8");
ArmAngle = cast((out.AAngle.data)', "int8");

writematrix(BaseAngle,'BaseAngle.txt','Delimiter',',');
writematrix(ArmAngle,'ArmAngle.txt','Delimiter',',')