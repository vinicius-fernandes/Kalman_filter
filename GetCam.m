function [PosX,PosY,Theta] = GetCam(Simulated)
sigma= 0.001;
sigma2=  0.004;
PosX= Simulated(1)+rndgaussian(0.0,sigma);
PosY = Simulated(2)+rndgaussian(0.0,sigma);
Theta = Simulated(3)+rndgaussian(0.0,sigma2);
Theta = NormalizedAngle(Theta);