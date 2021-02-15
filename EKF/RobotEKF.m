function [PosXKalman,PosYKalman,ThetaKalman] = RobotEKF(z, rates, dt)
%
%
persistent H R Q
persistent x P
persistent firstRun


if isempty(firstRun)
  H = eye(3);
    Q = [ 0.00001^2  0       0;
        0       0.00001^2  0;
        0       0       0.00001^2 ];
  R = diag([0.005^2,0.005^2,0.005^2]);

  x = [0 0 0].';  
  P = 0.1*eye(3);
  
  firstRun = 1;  
end

%Compute jacobian
A = Ajacob(x, rates, dt);

%Predict state and covariance
xp = fx(x, rates, dt);
Pp = A*P*A' +Q ;

%Compute kalman gain
K = (Pp*H')/(R+ H*Pp*H.');

%Calculation of the estimate
x = xp + K*(z - xp);

%Calculation of the covariance
P = Pp - K*H*Pp;


PosXKalman    = x(1);
PosYKalman    = x(2);
ThetaKalman   = x(3);


%Predict
%------------------------------
function xp = fx(xhat, rates, dt)
%
%
theta = xhat(3);
v=rates(1);
w=rates(2);


EPISLON=10E-3;
xdot= zeros(3,1);
%Avoid zero division
if(abs(w)<EPISLON)
xdot(1)=xhat(1)+ v * dt * cos(theta + (w * dt / 2.0));
xdot(2)=xhat(2)+ v * dt * sin(theta + (w * dt / 2.0));
else
xdot(1)=xhat(1)+2.0 * (v / w) * cos(theta + (w * dt / 2.0)) * sin(w * dt / 2);
xdot(2)=xhat(2)+2.0 * (v / w) * sin(theta + (w * dt / 2.0)) * sin(w * dt / 2);
end
xdot(3) = theta + w*dt;


xdot(3)=xdot(3);

xp = xdot;


%------------------------------
function A = Ajacob(xhat, rates, dt)
%Jacobian matrix of partial derivates of f to x 
%
EPISLON=10E-3;
A = zeros(3, 3);
theta = xhat(3);
v=rates(1);
w=rates(2);

A(1,1)=1;
A(1,2)=0;
%Avoid zero division
if(abs(w)<EPISLON)
    A(1,3)= -v * dt * sin(theta + (w * dt / 2.0));
else
    A(1,3)= -2.0 * (v / w) * sin(theta + (w * dt / 2.0)) * sin(w * dt / 2.0);
end

A(2,1)=0;
A(2,2)=1;
if(abs(w)<EPISLON)
    A(2,3)= v * dt * cos(theta + (w * dt / 2.0));
else
    A(2,3)= 2.0 * (v / w) * cos(theta + (w * dt / 2.0)) * sin(w * dt / 2.0);
end

A(3,1)=0;
A(3,2)=0;
A(3,3)=1;





