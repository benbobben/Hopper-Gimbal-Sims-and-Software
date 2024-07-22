%% Using the kinematic version of the maximum principle.
%  Introduce k and b into alpha and beta to get an accurate time calc

clf
clear 
clc
%Constants
m      =  .097;
g      =  -9.81;
ks     = 0;    % .224;
f      = 0;

% thrust = 10.12*m;    % (N) Thrust from compressor 
thrust = 10.12*m;
grav_F = g*m;        % (N)

u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
u_ON  =  thrust + grav_F;        % Net Force on body with thrust
a     = .41;                       % Desired Height

% System Dynamics
A = [ 0   1; 
      ks/m f/m  ];

B = [0; 
     1];

C = [1 0];

D = 0;

I = eye(2);

Tend = 30;  
dt  = .01;
t   =  0:dt:Tend;

X      = zeros([length(t),2]); % Position and Velocity
X(1,:) = [0,0];

% Time Calcs - Max Principal equations ; % a = desired position
b    = X(1,2);% b = current Velocity 
c    = X(1,1);% c = current position

beta = u_ON/m; %Acceleration With Thruster On  (Gravity and Thruster)
alpha= u_OFF/m; %Acceleration with Thruster OFF (Just Gravity)

tau  = -(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2))/(beta^2 + alpha*beta)
T    =   b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2)))/(alpha*(beta^2 + alpha*beta))


u    = zeros(length(t),1); % PWM  -- at time t --> u = 1 or 0

maxErr = 0.001; %Maximum distance from desired height
ti     = 0;  

for k = 1:length(t)-1
    posi = X(k,1);
    if (ti < tau)
        u(k) = 1;
        ti = ti+dt;
    elseif (tau < ti) && (ti < T)
        u(k) = 0;
        ti   = ti+dt;
    elseif (X(k,2) < 0) && (a - posi > maxErr)
         b = X(k,2); % b = initial Velocity 
         c = X(k,1); % c = initial position

         % beta  = -ks/m * c - f/m * b + u_ON/m;
         % alpha =  ks/m * c + f/m * b + u_OFF/m;
         
         %New Solution for tau and T
         tau =  -(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2))/(beta^2 + alpha*beta)
         T   =   b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2)))/(alpha*(beta^2 + alpha*beta))
         ti = 0;
    end

    if u(k) == 1
        ui = beta;
    else 
        ui = -alpha;
    end

    X(k+1,:) = (I+A*dt)*X(k,:)' + B*dt*(ui);
end

% error = X(k,1) - d;

control = u;

figure(1)
subplot(311)
plot(t,control)
title('Control')
xlabel("Time")
ylabel("Control (N)")
xlim([0 Tend])
ylim([0 1])


subplot(312)
plot(t,X(1:length(X),1))
title('Position Vs Time')
xlim([0 Tend])
% ylim([-1.1 1.1])
xlabel('time')
ylabel("Position (m)")

subplot(313)
plot(t,X(1:length(X),2))
title('Velocity vs Time')
xlabel("Time")
ylabel("Vel (m/s)")
xlim([0 Tend])
ylim([-5 5])

time_open = sum(u)*dt;


X_plot = zeros([length(t),2]);
r = 1;
dim =10;


% stableHertz = sum(u(end- 1/dt:end))
% timeOpenPerSec = dt*stableHertz
% 
% lastSecondPosition = X(end- 6*1/dt:end,1);
% averagePosStable = sum(lastSecondPosition)/length(lastSecondPosition)

% 
% for i = 1:5:length(t)-1
%     X_plot(i+1,:) = (I+A*dt)*X_plot(i,:)' + B*dt*u(i);
%     origin_y = X(i+1,1);
%     figure(2)
%     current_time = num2str(t(i), '%.1f');
%     title(current_time)
%     fill(r*cos(0:.1:2*pi),r*sin(0:.1:2*pi) + origin_y,'k')
%     axis([-dim dim -dim dim])
%     hold off
%     pause(.0001)
% end



