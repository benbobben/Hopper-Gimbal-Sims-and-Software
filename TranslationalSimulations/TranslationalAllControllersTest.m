%% TranslationalControllerTest.m - last edit 7-22-24
% This file has a numerical integrator loop that is used to test the
% translational (hopper) system. Constants are defined at the beginning.
% Define the system matricies. Functions created for the hysteresis, PIDPWM
% and MaxPrinciple controllers can be called to test the control laws. This
% allows one to test the controllers with different constants i.e. Gravity
% of the moon instead of the earth. 
% This file also contains functions for plotting the results in different
% manners. 
clear 
clc 
clf
close all
%Consts
m       =   0.097; %kg
g       =  -9.81 ;%m/s^2
ks      =   .223; %N/m
b       =   0;    %N/ms
accel_F = 10.21;     %acceleration of vehicle from compressor 

x_0 = [1 1];

% desired_Height = 0.4165; %m
desired_Height =  4; %m
desiredVel     =   0;
maxErr         =  .1; % After reching height 

% External Acceleration on body
alpha  = -g;         %Just gravity
beta   = g+accel_F;  %Firing thruster and gravity
% Force Values
u_ON   = (accel_F + g)*m;%N
u_OFF  = g*m;           %N

% System Dynamics ; Initialize system
A = [ 0   1; 
      -ks b/m  ];

% A = [ 0    1; 
%       -sqrt(.1)  0  ];

B = [ 0; 
      1];

C = [1 0];
D = 0;

I = eye(2);

Tend = 10;  
dt  = .01;
t   =  0:dt:Tend;

%%%%%%%%%%%%%%%%%
% Place Holder Arrays
X_Hys = zeros([length(t),2]); % Position and Velocity
X_Hys(1,:) = x_0; 

%%%%
%Tuning for Hysterisis consists of Finding the right max error
X_PID = zeros([length(t),2]); % Position and Velocity
X_PID_Cont = zeros([length(t),2]);
X_PID(1,:) = x_0; 
X_PID_Cont(1,:) = x_0; 

syms s
p = [-10 -2 -1];
pol = expand((s-p(1))*(s-p(2))*(s-p(3)));
pc = coeffs(pol);

kp   = double(pc(2));
ki   = double(pc(1));
kd   = double(pc(3));
% Initializing PID Sim

% kp    = 30;
% ki    = 20;
% kd    = 15;

%Discrete Func
int_Err    = 0;
period     = .05;
Ts         = 0;
duty_Cycle = 1;
u_Last     = 0 ; 
previous_Error = desired_Height;

%Continuous Func
int_Err_cont        = 0;
previous_Error_Cont = desired_Height;

%%%%%%%%%%%%% Maximum principle parameters
X_MPH = zeros([length(t),2]); % Position and Velocity
X_MPH(1,:) = x_0; 
b    = X_MPH(1,2);% b = current Velocity 
c    = X_MPH(1,1);% c = current position
a    = desired_Height;
% tau  = -(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2))/(beta^2 + alpha*beta);
% T    =   b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2)))/(alpha*(beta^2 + alpha*beta));

X1_T = desired_Height;
X2_T = 0;
va_0 = X_MPH(1,2);
xa_0 = X_MPH(1,1);
u1 = beta;
u2 = alpha;
c = A(2,1);
tau = log((u1 + c*xa_0 - c^(1/2)*va_0)/(u1 + u2) + ((u2 - X1_T*c + X2_T*c^(1/2))*(2*u1*u2 + X2_T^2*c + c*va_0^2 + c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0))/(2*(u1 + u2)*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)
T   = log((2*u1*u2 + X2_T^2*c + c*va_0^2 + c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0)/(2*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)
               
u_Hys = zeros(length(t),1); % PWM  -- at time t --> u = 1 or 0
u_Disc_PID = zeros(length(t),1);
u_Cont_PID = zeros(length(t),1);
u_MPH = zeros(length(t),1);%% Main Loop
Tend = real(T)
for k = 1:length(t)-1
    %Get control input (ON or OFF) for each system from functions
    u_Hys_d(k)                              = hysterisis_Hopper(X_Hys(k,:),desired_Height,maxErr);
                                                                       %(kp,kd,ki,duty_Cycle,period,X_k,d,desiredVel,int_Err,previous_Error,dt,t_k,Ts,u_ON)
    [u_Disc_PID(k), int_Err,duty_Cycle,Ts,previous_Error,u_Last]= PIDPWM_Hopper(kp,kd,ki,duty_Cycle,period,X_PID(k,:),desired_Height,desiredVel,int_Err,previous_Error,dt,t(k),Ts,accel_F,u_Last);

    [u_Cont_PID(k),int_Err_cont,previous_Error_Cont]          = PID_Cont_Hopper(kp,kd,ki,X_PID_Cont(k,:),desired_Height,desiredVel,int_Err_cont,previous_Error_Cont,dt);
    [u_MPH(k),tau,T]                      = hysterisisXMaxPrincipal_Hopper(X_MPH(k,:),t(k),desired_Height,maxErr,alpha,beta,tau,T,A(2,1));

    u_Hys(k)      = u_Hys_d(k) * accel_F + g;
    u_Disc_PID(k) = u_Disc_PID(k) * accel_F + g;
    u_MPH(k)      = u_MPH(k) * accel_F + g;

    X_Hys(k+1,:)      = (I+A*dt)*X_Hys(k,:)'      + B*dt*u_Hys(k);
    X_PID_Cont(k+1,:) = (I+A*dt)*X_PID_Cont(k,:)' + B*dt*(u_Cont_PID(k)+g);

    X_PID(k+1,:)      = (I+A*dt)*X_PID(k,:)'      + B*dt*u_Disc_PID(k);
    X_MPH(k+1,:)      = (I+A*dt)*X_MPH(k,:)'      + B*dt*u_MPH(k);
end

%Choose what information you want based on what you're testing

% getPosGraph(X_Hys,t,Tend)
% getGraphs(X_Hys,u_Hys,t,Tend,"Hysterisis",desired_Height )
% getGraphs(X_PID_Cont,u_Cont_PID,t,Tend,"PID Cont",desired_Height)
% getGraphs(X_PID,u_Disc_PID,t,Tend,"PID",desired_Height)
getGraphs(X_MPH,u_MPH,t,Tend,"MPH",desired_Height)

% compare2(X_Hys,X_PID,u_Hys,u_Disc_PID,t,Tend,"Hys","PID",desired_Height)
% name_All = ["Hys","Disc","MPH"];
% getOverlayGraph([X_Hys,X_PID,X_MPH],[u_Hys,u_Disc_PID,u_MPH],t,Tend,name_All,desired_Height)
% 
% [stableHertz_Hys,timeOpenPerSec_Hys,averagePosStable_Hys] = getStats(X_Hys,u_Hys,dt)
% [stableHertz_PID,timeOpenPerSec_PID,averagePosStable_PID] = getStats(X_PID,u_Disc_PID,dt)
% [stableHertz_MPH,timeOpenPerSec_MPH,averagePosStable_MPH] = getStats(X_MPH,u_MPH,dt)

function [u] = hysterisis_Hopper(X_k, d,maxErr) 
    %Given Current position, final desired position, and max Error
    X_k(1,1) = round(X_k(1,1) , 2 , "significant" );
    if (X_k(1,1) < d) || (X_k(1,2) < 0) &&  (X_k(1,1) < d + maxErr)
        u = 1;
        % ui = beta;
    else
        % ui = alpha;
        u = 0;
    end
end

%% Control Functions


function [u_Cont,int_Err,previous_Error] = PID_Cont_Hopper(kp,kd,ki,X_k,d,desiredVel,int_Err,previous_Error,dt)
    % Given The coefficients of PID Controller, duty cycle (%), period (s)
    % current states, desired position, desired velocity, and the
    % cumulated Error
    % Helpful values at time step
    posi     = X_k(1,1);
    % veli     = X_k(1,2);
    

    pos_Err  = d - posi;
    % vel_Err  = desiredVel - veli;
    der_Err = (-previous_Error+pos_Err)/dt;
    int_Err  = int_Err + pos_Err;
    
    % u_Cont    =  pos_Err*kp + kd*vel_Err/dt +ki*int_Err*dt;

    u_Cont    =  pos_Err*kp + kd*der_Err + ki*int_Err*dt;

    previous_Error = pos_Err;
    % fprintf("Position Error = " + pos_Err  + "; U = " + u_Cont + "Der_Err = " + der_Err + "\n")
end 

function [u_Disc,int_Err,duty_Cycle,Ts,previous_Error,u_Last] = PIDPWM_Hopper(kp,kd,ki,duty_Cycle,period,X_k,d,desiredVel,int_Err,previous_Error,dt,t_k,Ts,u_ON,u_Last)
    % Given The coefficients of PID Controller, duty cycle (%), period (s)
    % current states, desired position, desired velocity, and the
    % cumulated Error
    % Pulse width is the duty cycle times the period
    %Helpful values at time step
    posi     = X_k(1,1);

    pos_Err  = d - posi;
    der_Err = (-previous_Error+pos_Err)/dt;
    int_Err  = int_Err + pos_Err;

    % veli     = X_k(1,2);
    % vel_Err  = desiredVel - veli;
    % u_Cont    =  pos_Err*kp + kd*vel_Err/dt +ki*int_Err*dt;

    u_Cont    =  pos_Err*kp + kd*der_Err +ki*int_Err*dt;

    if     t_k< duty_Cycle*period + Ts
        %If in the duty cycle, fire thruster
        u_Disc = 1;
    elseif t_k < period + Ts
        %If still within period and outside duty cycle turn off thruster
        u_Disc = 0;
    else
        %If outside the current period Recalculate duty cycle
        param = (u_Last + u_Cont)/(u_ON) * 1;
        if param >= 1
            duty_Cycle  =  1;
        elseif param < 0 
            duty_Cycle  =  0;    
        else
            duty_Cycle  =  param;
        end
        u_Disc = 1;
        Ts = t_k;
    end
    previous_Error = pos_Err;
    u_Last = u_Cont;
end 

% function [X,u] = hysterisis_Hopper(t,dt,d,maxErr,alpha,beta,d_Alter)
%     distanceMP_StopFiring = 0.4165;
%     for k = 1:length(t)-1
% 
%         if (X(k,1) < d/15)
%             u(k) = 1;
%             ui = beta;
%         elseif (X(k,2) < 0) &&  (X(k,1) - maxErr< d)
%             u(k) = 1;%posi < a
%             ui = beta;
%         else
%             ui = -alpha;
%             u(k) = 0;
%         end
% 
%         X(k+1,:) = (I+A*dt)*X(k,:)' + B*dt*ui;
% 
%     end
% 
% end


function [u,tau,T] = hysterisisXMaxPrincipal_Hopper(X_k,t_k,d,maxErr,alpha,beta,tau,T,w)

    % Time Calcs - Max Principal equations
    % X1_T_Val = 1;
    % X2_T_Val = 0;
    % va_0_Val = 0 
    % xa_0_Val = 0
    % beta     = .31
    % alpha    = 9.81 

    X1_T   = d;
    X2_T   = 0;
    maxErr = .1;
    va_0   = X_k(1,2);
    xa_0   = X_k(1,1);
    
    c      = w;
    u1     = beta;
    u2     = alpha;

    %Currrent Issue, Undershooting for all tau T besides first
    % b = X_k(1,2); % b = initial Velocity 
    % c = X_k(1,1); % c = initial position

    if (t_k < tau)
        u = 1;
    elseif (tau < t_k) && (t_k  < T)
        u = 0;
    elseif (va_0 < 0)  && (xa_0 < d + maxErr)
        tau = t_k + log((u1 + c*xa_0 - c^(1/2)*va_0)/(u1 + u2) + ((u2 - X1_T*c + X2_T*c^(1/2))*(2*u1*u2 + X2_T^2*c + c*va_0^2 + c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0))/(2*(u1 + u2)*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)
        T   = t_k + log((2*u1*u2 + X2_T^2*c + c*va_0^2 + c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0)/(2*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)
        tau = real(tau);
        T   = real(T);
        u   = 1;
    else
        u = 0;
    end

end

%% Graphs of system

function getPosGraph(X,t,Tend)
    figure('Name',"Position Vs Time")  
    fontsize(16,"points")
    arrayPosition = X(1:length(X),1);
    plot(t,arrayPosition)
    xlim([0 Tend])
    ylim([min(arrayPosition), max(arrayPosition)+.005])
    title('Position Vs Time (m)')
    xlabel("Time (s)")
    ylabel("Position (m)")
end

function getGraphs(X,control,t,Tend,name,target)
    figure('Name',name)  
    arrayPosition = X(1:length(X),1);
    arrayVelocity = X(1:length(X),2);

    % Poisition Vs time
    subplot(311)
    plot(t,arrayPosition,"LineWidth",4)
    fontsize(16,"points")
    title('Position Vs Time')
    hold on
    plot(t,ones(length(t))*target,'-.r')
    xlim([0 Tend])
    ylim([min(arrayPosition), max(arrayPosition)+.5])
    xlabel('time')
    ylabel("Position (m)")
    legend("Simulation","Target")
    
    %Velocity Vs Time
    subplot(312)
    fontsize(16,"points")
    plot(t,arrayVelocity)
    xlim([0 Tend])
    ylim([min(arrayVelocity)-.01, max(arrayPosition)+.01])
    title('Velocity vs Time')
    xlabel("Time")
    ylabel("Velocity (m/s)")

    % Control input vs time  
    subplot(313)
    fontsize(16,"points")
    plot(t,control)
    xlim([0 Tend])
    ylim([min(control)-.5, max(control)+.5])
    title('Control')
    xlabel("Time")
    ylabel("Control")
end

function getOverlayGraph(X_All,U_All,t,Tend,name_All,d)
    figure('Name',"Overlay Graph of X1 X2")
    
    % subplot(311)
    % figure(6)
    hold on
    for k = 0:width(X_All)/2 - 1
        plot(t,X_All(:,1+k*2),'color',rand(1,3))
        
    end
    plot(t,ones(length(t))*d,'color','r')
    xlim([0 Tend])
    fontsize(16,"points")
    ylim([min(X_All(:,1))-.2 max(X_All(:,1))+.2])
    title('Position Vs Time ')
    xlabel("Time (s)")
    ylabel("Position (m)")
    legend(name_All)

    % subplot(312)
    % for k = 1:width(X_All) - 1
    %     plot(t,U_All(:,k),"r")
    %     hold on
    % end
    % xlim([0 Tend])
    % ylim([min(U_All(:,1))-.1 max(U_All(:,1))+.1])
    % title(name1 + " Control" )
    % xlabel("Time (s)")
    % ylabel("Control (1||0)")
    % legend(name_All)
    % hold on
    % 
    % subplot(313)
    % for k = 0:width(X_All)/2 - 1
    %     plot(t,X_All(:,2+k*2),"r")
    %     hold on
    % end
    % xlim([0 Tend])
    % ylim([-.1 1.1])
    % title(name2 + " Control")
    % xlabel("Time (s)")
    % ylabel("Velocity m/s")
    % legend(name_All)
end

function compare2(X,X2,control1,control2,t,Tend,name1,name2,d)
    arrayPosition1 = X(1:length(X),1);
    arrayPosition2 = X2(1:length(X),1);
    figure('Name',"Overlay Graph of X1 X2")

    subplot(311)
    plot(t,arrayPosition1,"r")
    hold on
    plot(t,arrayPosition2,"b")
    xlim([0 Tend])
    ylim([min(arrayPosition1)-.5 max(arrayPosition1)+.5])
    title('Position Vs Time ')
    xlabel("Time (s)")
    ylabel("Position (m)")
    legend(name1 + " pos",name2 + " pos")

    subplot(312)

    plot(t,control1.*ones(length(control1),1),"r")
    xlim([0 Tend])
    ylim([-.1 1.1])
    title(name1 + " Control" )
    xlabel("Time (s)")
    ylabel("Control (1||0)")
    hold on

    subplot(313)
    plot(t,control2.*ones(length(control2),1),"b")
    xlim([0 Tend])
    ylim([-.1 1.1])
    title(name2 + " Control")
    xlabel("Time (s)")
    ylabel("Control (1||0)")
end
% Stats
function [stableHertz,timeOpenPerSec,averagePosStable] = getStats(X,u,dt)
    
    stableHertz = sum(u(end- 1/dt:end));
    timeOpenPerSec = dt*stableHertz;
    
    lastSecondPosition = X(end- 6*1/dt:end,1); %% EDIT 6 is a place holder
    averagePosStable = sum(lastSecondPosition)/length(lastSecondPosition);
end
% Animation
function getAnimation_Hopper(t,X,dim,timeSkip)
    r = 1;
    for i = 1:timeSkip:length(t)-1
        origin_y = X(i+1,1);
        figure(2)
        current_time = num2str(t(i), '%.1f');
        title(current_time)
        fill(r*cos(0:.1:2*pi),r*sin(0:.1:2*pi) + origin_y,'k')
        axis([-dim dim -dim dim])
        hold off
        pause(.0001)
    end
end

