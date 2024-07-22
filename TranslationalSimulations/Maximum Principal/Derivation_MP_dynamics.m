%% Derive MPH Model mathematics with Compatibility equations 

clear 
clc


%% Laplace Transform 
syms  w x d s x_0 xd_0 t %x_T xd_T T tau

X_DD = w^2*x + d
x_oS = d/(s*(s^2+w^2)) - (s*x_0)/(s^2+w^2) - xd_0/(s^2+w^2)

x_ot = ilaplace(x_oS)

solve(x_ot,t)

%% derivation (equ a is time 0-->tau equ b = time tau-->T)
% syms k m beta t x_0 xd_0 x_T xd_T T tau
clf
clear 
clc

m      =  .097;
g      =  -9.81;
k     = .01;    % .224;
f      = 0;

thrust = 10.12*m;    % (N) Thrust from compressor 
grav_F = g*m;        % (N)

u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
u_ON  =  thrust + grav_F;        % Net Force on body with thrust
beta = u_ON/m;
alpha = u_OFF/m;

%Boundary Conditions
x_0  = 0;
xd_0 = 0;
x_T  = .41;                       % Desired Height
xd_T = 0;

Tend = 30;  
dt  = .01;
t   =  0:dt:Tend;

X      = zeros([length(t),2]); % Position and Velocity
Xd      = zeros([length(t),2]); % Position and Velocity

w = sqrt(k/m);

for i = 1:length(t)-1
    X(i,1) =   beta/(w^2)  * (1-cos(w*t(i))) + x_0*cos(w*t(i))     + xd_0*(1/w)*sin(w*t(i));
    X(i,2) =  -alpha/(w^2) * (1-cos(w*t(i))) + x_T*cos(w*t(i))     + xd_T*(1/w)*sin(w*t(i));
    
    Xd(i,1) =  beta /(w)   * (sin(w*t(i)))   - x_0*w*sin(w*t(i))   + xd_0*cos(w*t(i));
    Xd(i,2) = -alpha/(w)   * (sin(w*(t(i)))) - x_T*w*sin(w*(t(i))) + xd_T*cos(w*(t(i)));
end
% xd_a =   -beta /k   * w * (sin(w*tau))         - x_0*w*sin(w*tau)       + xd_0*cos(w*tau);
% xd_b =    alpha/k   * w * (sin(w*(tau-T_equ))) - x_T*w*sin(w*(tau-T_equ)) + xd_T*cos(w*(tau-T_equ));

figure(1)
plot(t,X(:,1))
hold on
plot(t,X(:,2))
legend("On_Pos","OFF_Pos")
title('Position for switching equations')
xlabel("Time")
ylabel("position (m)")
hold off
figure(2)
plot(t,Xd(:,1))
hold on
plot(t,Xd(:,2))
legend("On_Vel","OFF_Vel")
title('Velocity for switching equations')
xlabel("Time")
ylabel("Vel (m/s)")

%% Find T in terms of Tau

clc 
clear
syms k w m alpha beta t x_0 xd_0 x_T xd_T T tau w


x_a =   beta/(w^2)  * (1-cos(w*tau))       + x_0*cos(w*tau)           + xd_0*(1/w)*sin(w*tau);
x_b =  -alpha/(w^2) * (1-cos(w*(tau - T))) + x_T*cos(w*(tau - T))     + xd_T*(1/w)*sin(w*(tau - T));

T = simplify(solve(x_a==x_b,T))
T1 = tau - (log((alpha + beta - beta*cos(tau*w) - (2*alpha*beta - w^2*xd_T^2 - w^4*x_T^2 + beta^2*cos(tau*w)^2 + beta^2 - 2*beta^2*cos(tau*w) - 2*alpha*w^2*x_T + w^4*x_0^2*cos(tau*w)^2 + w^2*xd_0^2*sin(tau*w)^2 - 2*alpha*beta*cos(tau*w) - 2*beta*w^2*x_0*cos(tau*w)^2 + 2*alpha*w*xd_0*sin(tau*w) + 2*beta*w*xd_0*sin(tau*w) - beta*w*xd_0*sin(2*tau*w) + 2*alpha*w^2*x_0*cos(tau*w) + 2*beta*w^2*x_0*cos(tau*w) + w^3*x_0*xd_0*sin(2*tau*w))^(1/2) + w*xd_0*sin(tau*w) + w^2*x_0*cos(tau*w))/(x_T*w^2 + xd_T*w*1i + alpha))*1i)/w
T2 = tau - (log((alpha + beta - beta*cos(tau*w) + (2*alpha*beta - w^2*xd_T^2 - w^4*x_T^2 + beta^2*cos(tau*w)^2 + beta^2 - 2*beta^2*cos(tau*w) - 2*alpha*w^2*x_T + w^4*x_0^2*cos(tau*w)^2 + w^2*xd_0^2*sin(tau*w)^2 - 2*alpha*beta*cos(tau*w) - 2*beta*w^2*x_0*cos(tau*w)^2 + 2*alpha*w*xd_0*sin(tau*w) + 2*beta*w*xd_0*sin(tau*w) - beta*w*xd_0*sin(2*tau*w) + 2*alpha*w^2*x_0*cos(tau*w) + 2*beta*w^2*x_0*cos(tau*w) + w^3*x_0*xd_0*sin(2*tau*w))^(1/2) + w*xd_0*sin(tau*w) + w^2*x_0*cos(tau*w))/(x_T*w^2 + xd_T*w*1i + alpha))*1i)/w
 
dx_a =  beta /(w)   * (sin(w*tau))          - x_0*w*sin(w*tau)          + xd_0*cos(w*tau);
dx_b = -alpha/(w)   * (sin(w*(tau - T(1)))) - x_T*w*sin(w*(tau - T(1))) + xd_T*cos(w*(tau - T(1)));

tau1 = simplify(solve(dx_a==dx_b,tau))
tau11 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) - (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w
tau12 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) + (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w

dx_a =  beta /(w)   * (sin(w*tau))   - x_0*w*sin(w*tau)   + xd_0*cos(w*tau);
dx_b = -alpha/(w)   * (sin(w*(tau - T(2)))) - x_T*w*sin(w*(tau - T(2))) + xd_T*cos(w*(tau - T(2)));

tau2 = simplify(solve(dx_a==dx_b,tau))
tau21 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) - (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w
tau22 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) + (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w
 
%% Testing result
clc
clear
syms tau
m      =  .097;
g      =  -9.81;
k      = 0.224;    % .224;
w      = sqrt(k/m);
thrust = 10.12*m;    % (N) Thrust from compressor 
grav_F = g*m;        % (N)
u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
u_ON  =  thrust + grav_F;        % Net Force on body with thrust
beta = u_ON/m;
alpha = u_OFF/m;

%Boundary Conditions
x_0  = 0;
xd_0 = 0;
x_T  = .4;                       % Desired Height
xd_T = 0;
T1 = tau - (log((alpha + beta - beta*cos(tau*w) - (2*alpha*beta - w^2*xd_T^2 - w^4*x_T^2 + beta^2*cos(tau*w)^2 + beta^2 - 2*beta^2*cos(tau*w) - 2*alpha*w^2*x_T + w^4*x_0^2*cos(tau*w)^2 + w^2*xd_0^2*sin(tau*w)^2 - 2*alpha*beta*cos(tau*w) - 2*beta*w^2*x_0*cos(tau*w)^2 + 2*alpha*w*xd_0*sin(tau*w) + 2*beta*w*xd_0*sin(tau*w) - beta*w*xd_0*sin(2*tau*w) + 2*alpha*w^2*x_0*cos(tau*w) + 2*beta*w^2*x_0*cos(tau*w) + w^3*x_0*xd_0*sin(2*tau*w))^(1/2) + w*xd_0*sin(tau*w) + w^2*x_0*cos(tau*w))/(x_T*w^2 + xd_T*w*1i + alpha))*1i)/w
T2 = tau - (log((alpha + beta - beta*cos(tau*w) + (2*alpha*beta - w^2*xd_T^2 - w^4*x_T^2 + beta^2*cos(tau*w)^2 + beta^2 - 2*beta^2*cos(tau*w) - 2*alpha*w^2*x_T + w^4*x_0^2*cos(tau*w)^2 + w^2*xd_0^2*sin(tau*w)^2 - 2*alpha*beta*cos(tau*w) - 2*beta*w^2*x_0*cos(tau*w)^2 + 2*alpha*w*xd_0*sin(tau*w) + 2*beta*w*xd_0*sin(tau*w) - beta*w*xd_0*sin(2*tau*w) + 2*alpha*w^2*x_0*cos(tau*w) + 2*beta*w^2*x_0*cos(tau*w) + w^3*x_0*xd_0*sin(2*tau*w))^(1/2) + w*xd_0*sin(tau*w) + w^2*x_0*cos(tau*w))/(x_T*w^2 + xd_T*w*1i + alpha))*1i)/w
T = T2 
tau11 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) - (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w
tau12 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) + (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w

tau21 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) - (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w
tau22 = -(log(- ((w^4*x_0^2 - w^4*x_T^2 - 2*beta*w^2*x_0 - 2*alpha*w^2*x_T + w^2*xd_0^2 - w^2*xd_T^2 + 2*beta^2 + 2*alpha*beta)*1i)/(2*(alpha + beta)*(x_0*w^2*1i + xd_0*w - beta*1i)) + (w*(- 4*alpha^2*w^2*x_0^2 + 4*alpha^2*w^2*x_T^2 + 8*beta*alpha^2*x_0 - 8*beta*alpha^2*x_T - 4*alpha^2*xd_0^2 - 4*alpha*w^4*x_0^2*x_T + 4*alpha*w^4*x_T^3 - 4*beta*alpha*w^2*x_0^2 + 8*beta*alpha*w^2*x_0*x_T - 4*beta*alpha*w^2*x_T^2 - 4*alpha*w^2*x_T*xd_0^2 + 4*alpha*w^2*x_T*xd_T^2 + 8*beta^2*alpha*x_0 - 8*beta^2*alpha*x_T - 4*beta*alpha*xd_0^2 - 4*beta*alpha*xd_T^2 + w^6*x_0^4 - 2*w^6*x_0^2*x_T^2 + w^6*x_T^4 - 4*beta*w^4*x_0^3 + 2*w^4*x_0^2*xd_0^2 - 2*w^4*x_0^2*xd_T^2 + 4*beta*w^4*x_0*x_T^2 - 2*w^4*x_T^2*xd_0^2 + 2*w^4*x_T^2*xd_T^2 + 4*beta^2*w^2*x_0^2 - 4*beta*w^2*x_0*xd_0^2 + 4*beta*w^2*x_0*xd_T^2 - 4*beta^2*w^2*x_T^2 + w^2*xd_0^4 - 2*w^2*xd_0^2*xd_T^2 + w^2*xd_T^4 - 4*beta^2*xd_T^2)^(1/2))/(2*(alpha + beta)*(- x_0*w^2 + xd_0*w*1i + beta)))*1i)/w
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simplified equations change anything?

clc 
clear
syms k w m alpha beta t x_0 xd_0 x_T xd_T T tau w

x_a =   beta/(w^2)  * (1-cos(w*tau));
x_b =  -alpha/(w^2) * (1-cos(w*(tau - T))) + x_T*cos(w*(tau - T));

T = simplify(solve(x_a==x_b,T))
T1 = tau + acos((alpha + beta - beta*cos(tau*w))/(x_T*w^2 + alpha))/w
T2 = tau - acos((alpha + beta - beta*cos(tau*w))/(x_T*w^2 + alpha))/w

dx_a =  beta /(w)   * (sin(w*tau));
dx_b = -alpha/(w)   * (sin(w*(tau - T(1)))) - x_T*w*sin(w*(tau - T(1)));

tau1 = simplify(solve(dx_a==dx_b,tau))

tau11 = -(log(-(w^4*x_T^2 - 2*alpha*beta + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) - 2*beta^2 + 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w
tau12 = -(log((2*alpha*beta - w^4*x_T^2 + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) + 2*beta^2 - 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w


dx_a =  beta /(w)   * (sin(w*tau));
dx_b = -alpha/(w)   * (sin(w*(tau - T(2)))) - x_T*w*sin(w*(tau - T(2)));

tau2 = simplify(solve(dx_a==dx_b,tau))
tau21 = -(log(-(w^4*x_T^2 - 2*alpha*beta + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) - 2*beta^2 + 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w
tau22 = -(log((2*alpha*beta - w^4*x_T^2 + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) + 2*beta^2 - 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w
 

%%

clc
clear
syms tau
m      =  .097;
g      =  -9.81;
k      = 0.224;    % .224;
w      = sqrt(k/m);
thrust = 10.12*m;    % (N) Thrust from compressor 
grav_F = g*m;        % (N)
u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
u_ON  =  thrust + grav_F;        % Net Force on body with thrust
beta = u_ON/m;
alpha = u_OFF/m;

%Boundary Conditions
x_0  = 0;
xd_0 = 0;
x_T  = .4;                       % Desired Height
xd_T = 0;

T1 = tau + acos((alpha + beta - beta*cos(tau*w))/(x_T*w^2 + alpha))/w
T2 = tau - acos((alpha + beta - beta*cos(tau*w))/(x_T*w^2 + alpha))/w
T = T2;

tau11 = -(log(-(w^4*x_T^2 - 2*alpha*beta + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) - 2*beta^2 + 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w
tau12 = -(log((2*alpha*beta - w^4*x_T^2 + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) + 2*beta^2 - 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w

tau21 = -(log(-(w^4*x_T^2 - 2*alpha*beta + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) - 2*beta^2 + 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w
tau22 = -(log((2*alpha*beta - w^4*x_T^2 + w*(-x_T*(x_T*w^2 + 2*alpha)*(- x_T*w^2 + 2*beta)*(x_T*w^2 + 2*alpha + 2*beta))^(1/2) + 2*beta^2 - 2*alpha*w^2*x_T)/(2*beta*(alpha + beta)))*1i)/w
 




%% Just Integration
clc
clear

m      =  .097;
g      =  -9.81;
k      = 0.224;    % .224;
w      = sqrt(k/m);
thrust = 10.12*m;    % (N) Thrust from compressor 
grav_F = g*m;        % (N)
u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
u_ON  =  thrust + grav_F;        % Net Force on body with thrust
beta = u_ON/m;
alpha = u_OFF/m;

%Boundary Conditions
x_0  = 0;
xd_0 = 0;
x_T  = .4;                       % Desired Height
xd_T = 0;



syms w u
fun = @(t) t;
q = integral(fun,5,3)




%% TRASH BEYOND THIS POINT




% %% velocity x_Dot for a and b equations
% % xd_a = beta/k   * w * (sin(w*t)) + x_0*w*sin(w*t) - xd_0*cos(w*t);
% % xd_b = -alpha/m * w * (sin(w*t)) + x_0*w*sin(w*t) - xd_0*cos(w*t);
% 
% % xd_a =  beta/k   * w * (sin(w*tau)) + x_0*w*sin(w*tau) - xd_0*cos(w*tau);
% % xd_b = -alpha/k  * w * (sin(w*(tau-T))) + x_0*w*sin(w*(tau-T)) - xd_0*cos(w*(tau-T));
% 
% % simplify(diff(x_T,t))
% 
% 
% 
% %% Finding tau and T for the first specific case with knowns to test if works
% clc 
% clear 
% syms tau T
% m      =  .097;
% g      =  -9.81;
% k      = 0.000001;    % .224;
% w      = sqrt(k/m);
% thrust = 10.12*m;    % (N) Thrust from compressor 
% grav_F = g*m;        % (N)
% u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
% u_ON  =  thrust + grav_F;        % Net Force on body with thrust
% beta = u_ON/m;
% alpha = u_OFF/m;
% 
% %Boundary Conditions
% x_0  = 0;
% xd_0 = 0;
% x_T  = 2;                       % Desired Height
% xd_T = 0;
% 
% x_a =  beta /k * (1-cos(w*tau))     - x_0*cos(w*tau)     - xd_0*(1/w)*sin(w*tau);
% x_b = -alpha/k * (1-cos(w*(tau-T))) - x_T*cos(w*(tau-T)) - xd_T*(1/w)*sin(w*(tau-T));
% eq1 = x_a == x_b;
% 
% T = simplify(solve(eq1,T))
% % Solution of T in terms of Tau:
% T_equ = T(2)
% 
% xd_a =   beta /k   * w * (sin(w*tau))         + x_0*w*sin(w*tau)         - xd_0*cos(w*tau);
% xd_b =  -alpha/k   * w * (sin(w*(tau-T_equ))) + x_T*w*sin(w*(tau-T_equ)) - xd_T*cos(w*(tau-T_equ));
% eq2  = xd_a == xd_b;
% 
% tau_Sol = simplify(solve(eq2,tau))
% 
% %%
% % fsolve(@root2d,[0,0])
% 
% function F = root2d(T)
%     syms tau T
%     m      =  .097;
%     g      =  -9.81;
%     k      = .01;    % .224;
%     w      = sqrt(k/m);
%     thrust = 10.12*m;    % (N) Thrust from compressor 
%     grav_F = g*m;        % (N)
%     u_OFF =  -grav_F;                % Absolute Val External Force on body when no thrust
%     u_ON  =  thrust + grav_F;        % Net Force on body with thrust
%     beta = u_ON/m;
%     alpha = u_OFF/m;
% 
%     %Boundary Conditions
%     x_0  = 0;
%     xd_0 = 0;
%     x_T  = .41;                       % Desired Height
%     xd_T = 0;
% 
%     x_a =  beta/k  * (1-cos(w*tau)) - x_0*cos(w*tau) - xd_0*(1/w)*sin(w*tau);
%     x_b = -alpha/m * (1-cos(w*(tau-T))) - x_T*cos(w*(tau-T)) - xd_T*(1/w)*sin(w*(tau-T));
%     F(1) = x_a == x_b;
% 
%     xd_a =   beta/k   * w * (sin(w*tau))         + x_0*w*sin(w*tau)         - xd_0*cos(w*tau);
%     xd_b =  -alpha/k  * w * (sin(w*(tau-T))) + x_0*w*sin(w*(tau-T)) - xd_0*cos(w*(tau-T));
%     F(2) = xd_a == xd_b;
% 
%     % F(1) = exp(-exp(-(x(1)+x(2)))) - x(2)*(1+x(1)^2);
%     % F(2) = x(1)*cos(x(2)) + x(2)*sin(x(1)) - 0.5;
% end
% 
% 
% 
% %% Testing the k = 0 so it can match 
% clc 
% clear 
% syms tau T
% m      =  .097;
% g      =  -9.81;
% k      = 0.000001;    % .224;
% w      = sqrt(k/m);
% thrust = 10.12*m;    % (N) Thrust from compressor 
% grav_F = g*m;        % (N)
% u_OFF  =  -grav_F;                % Absolute Val External Force on body when no thrust
% u_ON   =  thrust + grav_F;        % Net Force on body with thrust
% beta   = u_ON/m;
% alpha  = u_OFF/m;
% 
% %Boundary Conditions
% x_0  = 0;
% xd_0 = 0;
% x_T  = .41;                       % Desired Height
% xd_T = 0;
% 
% x_a =  beta /k * (1-cos(w*tau))     + x_0*cos(w*tau)     + xd_0*(1/w)*sin(w*tau);
% x_b = -alpha/k * (1-cos(w*(tau-T))) + x_T*cos(w*(tau-T)) + xd_T*(1/w)*sin(w*(tau-T));
% eq1 = x_a == x_b;
% 
% T = simplify(solve(eq1,T))
% % Solution of T in terms of Tau:
% T_equ = T(2)
% 
% xd_a =   -beta /k   * w * (sin(w*tau))         - x_0*w*sin(w*tau)       + xd_0*cos(w*tau);
% xd_b =    alpha/k   * w * (sin(w*(tau-T_equ))) - x_T*w*sin(w*(tau-T_equ)) + xd_T*cos(w*(tau-T_equ));
% eq2  = xd_a == xd_b;
% 
% tau_Sol = simplify(solve(eq2,tau))
% tau = 0;
% -(995.3500*sin(0.0032*tau))  + (31498*(1 - (0.0316*cos(0.0032*tau) - 1.0316)^2)^(1/2)) 
% % solve(-(995.3500*sin(0.0032*tau))  + (31498*(1 - (0.0316*cos(0.0032*tau) - 1.0316)^2)^(1/2)) == 0,tau)
% 
% 
% 
% %% When ICs are zero; Only x(T)
% clc 
% clear 
% syms tau T
% m      =  .097;
% g      =  -9.81;
% k      = 0.000001;    % .224;
% w      = sqrt(k/m);
% thrust = 10.12*m;    % (N) Thrust from compressor 
% grav_F = g*m;        % (N)
% u_OFF  =  -grav_F;                % Absolute Val External Force on body when no thrust
% u_ON   =  thrust + grav_F;        % Net Force on body with thrust
% beta   = u_ON/m;
% alpha  = u_OFF/m;
% 
% x_T = 1
% 
% x_a =  beta / (m*w^2) * (1-cos(w*tau)) 
% x_b = -alpha/ (m*w^2) * (1-cos(w*(tau-T))) + x_T*cos(w*(tau-T)) 
% eq1 = x_a == x_b;
% T1 = solve(eq1,T)
% T1 = T1(1)
% xd_a =   beta /(m*w) * (sin(w*tau)) 
% xd_b =  -alpha/(m*w) * (sin(w*(tau-T1))) - x_T*w*sin(w*(tau-T1)) 
% eq2  = xd_a == xd_b;
% 
% solve(eq2,tau)
% 
% 
% %% By hand??????dfaksdj
% 
% clc
% clear
% % syms beta alpha w x_T T tau m
% m      =  .097;
% g      =  -9.81;
% k      = 0.000001;    % .224;
% w      = sqrt(k/m);
% thrust = 10.12*m;    % (N) Thrust from compressor 
% grav_F = g*m;        % (N)
% u_OFF  =  -grav_F;                % Absolute Val External Force on body when no thrust
% u_ON   =  thrust + grav_F;        % Net Force on body with thrust
% beta   = u_ON/m;
% alpha  = u_OFF/m;
% x_T = 1
% equ1 = (beta + alpha)/(alpha+x_T*m*w^2) - cos(w*tau-w*T) - beta*cos(tau*w) == (beta)/(alpha+x_T*m*w^2)*sin(w*T)+cot(w*T)-cot(w*tau)
% theta1sol = solve(equ1,T,'returnconditions',true)
% 
% 
% 1.0316- cos(w*tau-w*T) - beta*cos(tau*w) == 0.0316*sin(w*T)+cot(w*T)-cot(w*tau)
% 
% 
% %% Simplify
% 
% clc
% clear
% 
% syms tau T
% % equ1 = 1 == cos(tau-T) + cos(tau)
% % T1 = solve(equ1,T)
% % equ2 = sin(tau) + 2*sin(tau-T1) == 0
% % tau = solve(equ2,tau)
% 
% eqns = [1 == cos(tau-T) + cos(tau), sin(tau) + 2*sin(tau-T) == 0]
% S = solve(eqns, [tau T]) %This works to find negative and imaginary values of T and tau
% 
% %% Now Introducing constant terms and no initail conditions
% clc
% clear
% 
% syms k w m alpha beta t x_0 xd_0 x_T xd_T T tau
% % syms T tau
% m      =  .01;
% g      =  -9.81;
% k      = .22;    % .224;
% w      = sqrt(k/m);
% thrust = 10.12*m;    % (N) Thrust from compressor 
% grav_F = g*m;        % (N)
% u_OFF  =  -grav_F;                % Absolute Val External Force on body when no thrust
% u_ON   =  thrust + grav_F;        % Net Force on body with thrust
% beta   = u_ON/m;
% alpha  = u_OFF/m;
% x_T  = .41
% 
% % x_a  =  beta/(m*w^2) * (1-cos(w*tau)) 
% % x_b  = -alpha/(m*w^2) * (1-cos(w*(tau-T))) + x_T*cos(w*(tau-T)) 
% % 
% % xd_a =   beta/(m*w) * (sin(w*tau)) 
% % xd_b =  -alpha/(m*w) * (sin(w*(tau-T))) - x_T*w*sin(w*(tau-T)) 
% 
% x_a  =  beta/(w^2)  * (1-cos(w*tau))
% x_b  = -alpha/(w^2) * (1-cos(w*(tau-T))) - x_T*cos(w*(tau-T))
% 
% xd_a =  beta/(w)  * (sin(w*(tau))) 
% xd_b = -alpha/w * (sin(w*(tau-T))) - x_T*w*sin(w*(tau-T))
% 
% eqns = [x_a == x_b, xd_a == xd_b]
% S2   = solve(eqns, [tau T])
% 
% 
% 
% 
% %% Another Method:
% 
% clc
% clear
% 
% syms k w m alpha beta t x_0 xd_0 x_T xd_T T tau
% 
% equ = (beta + alpha)/(alpha + x_T*m*w^2) == cos(1/w * asin((beta/(x_T*m*w-alpha))*sin(w*tau)) + beta*cos(tau*w))
% 
% solve(equ,tau,'returnconditions',true)
% % 





