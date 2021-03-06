addpath '.\EKF'
addpath '.\UKF'
clear all
close all

%Define the desired number of samples
Nsamples = 500;

%Define the output matrixes
Simulator = zeros(Nsamples, 3);
Camera = zeros(Nsamples, 3);
EKF = zeros(Nsamples,3);
UKF = zeros(Nsamples,3);


%Load the simulator data
load simulatorOut

%Define the dt 
dt = 1/30;

%Run the EKF and UKF for each sample
for k=1:Nsamples
    v=out.inputV.Data(k,:);
    w=out.inputW.Data(k,:);
  Simulator(k, :) = [ out.state.Data(k,1),out.state.Data(k,2),wrapToPi(out.state.Data(k,3))];
  Camera(k,:)=[out.Camera.Data(k,1),out.Camera.Data(k,2),wrapToPi(out.Camera.Data(k,3))];
  [PosXEKF,PosYEKF,ThetaEKF]=RobotEKF([out.Camera.Data(k,1),out.Camera.Data(k,2),out.Camera.Data(k,3)].',[v,w].',dt);
  [PosXUKF,PosYUKF,ThetaUKF]=RobotUKF([out.Camera.Data(k,1),out.Camera.Data(k,2),out.Camera.Data(k,3)].',[v,w].',dt);
  EKF(k,:)=[PosXEKF,PosYEKF,wrapToPi(ThetaEKF)];
  UKF(k,:)=[PosXUKF,PosYUKF,wrapToPi(ThetaUKF)];
end 



t = 0:dt:Nsamples*dt-dt;
RMSEekf= sqrt(mean((EKF(:,1) - Simulator(:,1)).^2));
RMSEukf= sqrt(mean((UKF(:,1) - Simulator(:,1)).^2));
RMSEmeasure= sqrt(mean((Camera(:,1) - Simulator(:,1)).^2));
fprintf("###############################################\n");
fprintf("RMSE EKF POS X: %d \n",RMSEekf);
fprintf("RMSE UKF POS X: %d \n",RMSEukf);
fprintf("RMSE MEASURE POS X: %d \n",RMSEmeasure);

RMSEekf= sqrt(mean((EKF(:,2) - Simulator(:,2)).^2));
RMSEukf= sqrt(mean((UKF(:,2) - Simulator(:,2)).^2));
RMSEmeasure= sqrt(mean((Camera(:,2) - Simulator(:,2)).^2));
fprintf("###############################################\n");
fprintf("RMSE EKF POS Y: %d \n",RMSEekf);
fprintf("RMSE UKF POS Y : %d \n",RMSEukf);
fprintf("RMSE MEASURE POS Y: %d \n",RMSEmeasure);

RMSEekf= sqrt(mean((EKF(:,3) - Simulator(:,3)).^2));
RMSEukf= sqrt(mean((UKF(:,3) - Simulator(:,3)).^2));
RMSEmeasure= sqrt(mean((Camera(:,3) - Simulator(:,3)).^2));
fprintf("###############################################\n");
fprintf("RMSE EKF THETA: %d \n",RMSEekf);
fprintf("RMSE UKF THETA: %d \n",RMSEukf);
fprintf("RMSE MEASURE THETA: %d \n",RMSEmeasure);
fprintf("###############################################\n");

%Output plot
figure
plot(Simulator(:,1),Simulator(:,2),'b-','DisplayName','Simulator')
title("PosX x PosY")
xlabel('Pos X')
ylabel('Pos Y')
hold on
%plot(Camera(:,1), Camera(:,2),'DisplayName','Camera')
plot(EKF(:,1), EKF(:,2),'r-','DisplayName','EKF')
plot(UKF(:,1), UKF(:,2),'g-','DisplayName','UKF')
legend
hold off

figure
%plot(t,Simulator(:,1),'b-','DisplayName','Simulator')
title("PosX ")
xlabel('Time(s)')
ylabel('Pos X')
hold on
plot(t, Camera(:,1),'b-','DisplayName','Camera')
plot(t, EKF(:,1),'r-','DisplayName','EKF')
plot(t, UKF(:,1),'g-','DisplayName','UKF')
legend
hold off

figure
%plot(t,Simulator(:,2),'b-','DisplayName','Simulator')
title("PosY ")
xlabel('Time(s)')
ylabel('Pos Y')
hold on
plot(t, Camera(:,2),'b-','DisplayName','Camera')
plot(t, EKF(:,2),'r-','DisplayName','EKF')
plot(t, UKF(:,2),'g-','DisplayName','UKF')
legend
hold off


figure
%plot(t,Simulator(:,3),'b-','DisplayName','Simulator')
title("Theta")
xlabel('Time (s)')
ylabel('Theta')
hold on
plot(t, wrapToPi(Camera(:,3)),'b-','DisplayName','Camera')
plot(t, EKF(:,3),'r-','DisplayName','EKF')
plot(t, UKF(:,3),'g-','DisplayName','UKF')
legend
hold off



