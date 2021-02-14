addpath '.\EKF'
addpath '.\UKF'
clear all
close all

%Define the desired number of samples
Nsamples = 500;

%Define the output matrixes
Simulator = zeros(Nsamples, 3);
Observer = zeros(Nsamples, 3);
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
  Simulator(k, :) = [ out.state.Data(k,1),out.state.Data(k,2),out.state.Data(k,3)];
  Observer(k,:)=[out.observer.Data(k,1),out.observer.Data(k,2),out.observer.Data(k,3)];
  [PosXEKF,PosYEKF,ThetaEKF]=RobotEKF([out.observer.Data(k,1),out.observer.Data(k,2),out.observer.Data(k,3)].',[v,w].',dt);
  [PosXUKF,PosYUKF,ThetaUKF]=RobotUKF([out.observer.Data(k,1),out.observer.Data(k,2),out.observer.Data(k,3)].',[v,w].',dt);
  EKF(k,:)=[PosXEKF,PosYEKF,ThetaEKF];
  UKF(k,:)=[PosXUKF,PosYUKF,ThetaUKF];
end 



t = 0:dt:Nsamples*dt-dt;
ERROR1= Simulator-EKF;
ERROR2= Simulator-UKF;

%Output plot
figure
plot(Simulator(:,1),Simulator(:,2),'b-','DisplayName','Simulator')
xlabel('Pos X')
ylabel('Pos Y')
hold on
%plot(Observer(:,1), Observer(:,2),'DisplayName','Observer')
plot(EKF(:,1), EKF(:,2),'r-','DisplayName','EKF')
plot(UKF(:,1), UKF(:,2),'g-','DisplayName','UKF')
legend
hold off

figure
plot(t,ERROR1(:,3),'DisplayName','Error1')
hold on
plot(t,ERROR2(:,3),'DisplayName','Error2')
legend
hold off
