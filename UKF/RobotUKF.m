function [PosXKalman,PosYKalman,ThetaKalman] = RobotUKF(z, rates, dt)
%
%
persistent Q R
persistent x P
persistent n m
persistent firstRun


if isempty(firstRun)
  Q = [ 0.00001^2  0       0;
        0       0.00001^2  0;
        0       0       0.00001^2 ];
     
R = diag([0.1^2,0.1^2,0.1^2]);


  x = [0 0 0]';  
  P = 10E-3*eye(3);
  
  n = 3;
  m = 3;
  
  firstRun = 1;  
end


[Xi,W] = SigmaPoints(x, P, 0);

fXi = zeros(n, 2*n+1);
for k = 1:2*n+1
  fXi(:, k) = fx(Xi(:,k), rates, dt);
end

[xp,Pp] = UT(fXi, W, Q);

hXi = zeros(m, 2*n+1);
for k = 1:2*n+1
  hXi(:, k) = hx(fXi(:,k));
end

[zp,Pz] = UT(hXi, W, R);

Pxz = zeros(n, m);
for k = 1:2*n+1
  Pxz = Pxz + W(k)*(fXi(:,k) - xp)*(hXi(:,k) - zp)';
end

K = Pxz/inv(Pz);

x = xp + K*(z - zp);
P = Pp - K*Pz*K';


PosXKalman   = x(1);
PosYKalman = x(2);
ThetaKalman   = x(3);


%------------------------------
function xp = fx(xhat, rates, dt)
%
%
theta = xhat(3);
v=rates(1);
w=rates(2);


EPISLON=10E-3;
xdot= zeros(3,1);
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
function yp = hx(x)
%
%
yp(1) = x(1);
yp(2) = x(2);
yp(3) =  x(3);