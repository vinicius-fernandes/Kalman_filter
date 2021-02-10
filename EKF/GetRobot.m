function [PosX,PosY,Theta] = GetRobot()

persistent k firstRun


if isempty(firstRun) 
  load ArsGyro
  k = 1;
  
  firstRun = 1;
end