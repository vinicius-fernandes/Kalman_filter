clear all
close all
Nsamples = 1000;
Simulator = zeros(Nsamples, 3);
Observer = zeros(Nsamples, 3);
Kalman = zeros(Nsamples,3);
load simulatorOut
dt = 1/30;
   v=0.1;
   w=  (20 * pi/ 180.0);
for k=1:Nsamples

  %disp("Simulator");
   %disp(SimulatedValue(3));
  Simulator(k, :) = [ out.state.Data(k,1),out.state.Data(k,2),out.state.Data(k,3)];
 % [PosX,PosY,Theta] = GetCam(SimulatedValue);
    %disp("Cam");
   %disp(Theta);
  Observer(k,:)=[out.observer.Data(k,1),out.observer.Data(k,2),out.observer.Data(k,3)];
  [PosXKalman,PosYKalman,ThetaKalman]=RobotEKF([out.observer.Data(k,1),out.observer.Data(k,2),out.observer.Data(k,3)].',[v,w].',dt);
  Kalman(k,:)=[PosXKalman,PosYKalman,ThetaKalman];
end 



t = 0:dt:Nsamples*dt-dt;

figure
plot(Simulator(:,1),Simulator(:,2))
title("Simulator")

figure
plot(Observer(:,1), Observer(:,2))
title("Observer")

figure
plot(Kalman(:,1), Kalman(:,2))
title("Kalman")

