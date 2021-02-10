function Simulated = RobotSimulator(v,w,dt)


EPSILON = 10E-3;
Simulated = zeros(3,1);
v= v+normrnd(0,0.001);
w=w+normrnd(0,0.001);
persistent firstRun PosXk PosYk Thetak

if isempty(firstRun)
    PosXk=0;
    PosYk=0;
    Thetak=0;
    firstRun = 1;  
end
disp(PosXk);
disp(PosYk);
disp(Thetak);
Thetak = NormalizedAngle(Thetak);

if(abs(w)<EPSILON)
   PosXk=PosXk+v*dt*cos(Thetak+ w*dt/2);
   PosYk=PosYk+v*dt*sin(Thetak+ w*dt/2);
else
   PosXk=PosXk+2*(v/w)*dt*cos(Thetak+ w*dt/2)*sin(w*dt/2);
   PosYk=PosYk+2*(v/w)*dt*sin(Thetak+ w*dt/2)*sin(w*dt/2);
end
 Thetak=Thetak+w*dt; 
 
Thetak = NormalizedAngle(Thetak);
Simulated(1)=PosXk;
Simulated(2)=PosYk;
Simulated(3)=Thetak;



