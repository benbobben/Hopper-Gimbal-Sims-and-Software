clc
clear
close all

% Consts
b      =  0;
ks      =  .224;
controlInput = 10.2; %N
g            = -9.81; %m/s^2
m            = .091; %kg

initialPos   = 0; 

finalPos     = .41;%m
desiredVel   = 0;

syms s

p = [-1 -2 -.5];                          % desired poles
pol = expand((s-p(1))*(s-p(2))*(s-p(3))); % Expanding the polynomial 
pc = coeffs(pol);                         % extracting coeficients of polynomial

% kp    = double(pc(2));
% ki    = double(pc(1));
% kd    = double(pc(3));
kp    = 30;
ki    = 0;
kd    = 0;
% ks = .224;

ks = 0 ;
% System Dynamics
A = [ 0   1; 
      ks/m   0 ];

B = [0; 
     1];

C = [1 0];
D = 0;

I = eye(2);

noiseGain = 0;
d    = finalPos; % Desired Height
% maxErr = .01; %Maximum distance from desired height

Tend = 20;  
dt   = .01;
t    =  0:dt:Tend;
ti   = 0;  
X     = zeros([length(t),2]);   % Position and Velocity
% X_Cont= zeros([length(t),2]); 
X(1,:)  = [initialPos,.01];  % b = X(1,2);% b = initial Velocity % c = X(1,1);% c = initial position
int_Err = 0;
u_Continuous = zeros(length(t),1);
u = zeros(length(t),1); % PWM  -- at time t --> u = 1 or 0
% u = ones(length(t),1)*controlInput;
Ts = 0;
param = zeros(length(t),1);
period = .05;
duty_Cycle = 1;
for k = 1:length(t)-1
    posi     = X(k,1);
    veli     = X(k,2);
    pos_Err  = d - posi;
    vel_Err  = desiredVel - veli;
    int_Err  = int_Err + pos_Err;
    
    u_Continuous(k)   =  pos_Err*kp + kd*vel_Err/dt + ki*int_Err*dt;
    
    if     t(k)< duty_Cycle*period + Ts
        %If in the duty cycle, fire thruster
        u(k) = controlInput;
    elseif t(k) < period + Ts
        %If still within period and outside duty cycle turn off thruster
        u(k) = 0;
    else
        %If outside the current period Recalculate duty cycle
        param(k) = mean(u_Continuous(k-period*100:k))/(controlInput-g);
        if param(k) >= .8
            duty_Cycle  =  1;
        else
            duty_Cycle  =  u_Continuous(k)/(controlInput);
            % duty_Cycle(k)  = u_Continuous(k-1) + u_Continuous(k)/controlInput;
        end
        u(k) = controlInput;
        Ts = t(k);
    end
    X(k+1,:) = (I+A*dt)*X(k,:)' + B*dt*(u(k) + g);
end


figure(1)
subplot(311)
plot(t,u)
title('Control Cont')
xlabel("Time")
ylabel("Control (N)")
xlim([0 Tend])
% ylim([-alpha-1 beta+1])

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
