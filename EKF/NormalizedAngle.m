function NewAngle = NormalizedAngle(theta)
%
%
if(theta>pi)
     NewAngle=theta-2*pi;
elseif(theta<-pi)
     NewAngle=theta+2*pi;  
else
    NewAngle = theta;
end