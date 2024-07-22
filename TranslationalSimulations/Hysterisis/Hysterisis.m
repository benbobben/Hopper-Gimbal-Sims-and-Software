clf
clear 
clc
% Initial Conditions
x0 = [0;  
      0]; 

%Consts
m = .097;
g = -9.81;
ks = .244
% control_Force = -(Pe-Pa)*Ae;
control_Force = 10.2;
% System Dynamics
A = [0 1; 
     ks/m 0];

B = [0; 
     1];

C = [1 0];
D = 0;

I = eye(2);




alpha = g; % External Force on body
beta = control_Force+alpha; % Net force on body with thrust
d    = .41; % Desired Height

Tend = 10;  
dt  = .01;
hz = 5;
t   =  0:dt:Tend;
X = zeros([length(t),2]); % Position and Velocity

u = zeros(length(t),1); % PWM  -- at time t --> u = 1 or 0
maxErr = .1;
ti = 0; 

for k = 1:length(t)-1

    if (X(k,1) < d) || (X(k,2) < 0) &&  (X(k,1) < d + maxErr)
        u(k) = 1;
        ui = beta;
    else
        ui = alpha;
        u(k) = 0;
    end
    
    X(k+1,:) = (I+A*dt)*X(k,:)' + B*dt*ui;
    
end


control = u;

figure(1)
subplot(311)
plot(t,control)
title('Control')
xlabel("Time")
ylabel("Control (N)")
xlim([0 Tend])
ylim([-.1 1.1])


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

% for i = 1:5:length(t)-1
% %     X_plot(i+1,:) = (I+A*dt)*X_plot(i,:)' + B*dt*u(i);
%     origin_y = X(i+1,1);
%     figure(2)
%     current_time = num2str(t(i), '%.1f');
%     title(current_time)
%     fill(r*cos(0:.1:2*pi),r*sin(0:.1:2*pi) + origin_y,'k')
%     axis([-dim dim -dim dim])
%     hold off
%     pause(.0001)
% end



%% Stats

stableHertz = sum(u(end- 1/dt:end))
timeOpenPerSec = dt*stableHertz

lastSecondPosition = X(end- 6*1/dt:end,1);
averagePosStable = sum(lastSecondPosition)/length(lastSecondPosition)


