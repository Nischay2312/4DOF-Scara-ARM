%Script to calculate endeffector force
pullH = 1*1e-2;
pipeD = 11.05*1e-3;
A1 = pi*pipeD/2*pipeD/2;
HoleVolume = A1 * pullH;

%PV = nRT
Temp = -15+273.15;
Rgas = 8.31446261815324;

AirDensity = 1.225;
AirMolarDensity = 28.97*1e-3;
moles = AirDensity * HoleVolume / AirMolarDensity; 

P1 = moles * Rgas * Temp/ HoleVolume;

outHoleD = 2.55*1e-3;
A2 = pi*(outHoleD/2)*(outHoleD/2);

P2 = P1*A1/A2;

Mmax = P2*A2/9.8;



