clc
clear

% Define Terms
syms t xa_0 va_0 r t2 u u1 u2 T w1 w2 x1(t) x2(t) xx1 xx2 dd TT uuu1 X1_T X2_T c
% uuu1 is the syms version of some arbitrary input

%Dynamics
A = [0   1;
     c  0]; % cannot handle a large spring constant (physics!)

B = [0 1]';

%Define Equations:
eqns = [diff(x1,t) == A(1,:)*[x1;x2] + B(1,:)*uuu1,  % Velocity     ----> Position
        diff(x2,t) == A(2,:)*[x1;x2] + B(2,:)*uuu1]; % Acceleration ----> Velocity

%Define initial Conditions
cond = [x1(0) == xa_0,   % Initial Position
        x2(0) == va_0]   % Initial Velocity

% Solve the differential Equation with initial conditions
[x1Sol(t),x2Sol(t)] = dsolve(eqns,cond) %General Equations For [ Position , Velocity ]  with respect to t

x = [expand(subs(x1Sol(t), [uuu1], [u1])); 
     expand(subs(x2Sol(t), [uuu1], [u1]))];

% Defining second equation --> same equation but with t2 as the final time
ytemp = subs(x,t,t2);
% Substituting first Acceleration, Beta, known into the Equation
yy = expand(subs(x,[xa_0 va_0 t u1],[ytemp(1) ytemp(2) t-t2 -u2]));

% u = u
% x = [ v1+t*v2+1/2*u1*t^2;v2+u1*t];
% yy = [v1-u1*t2^2+t*v2+2*u1*t*t2-1/2*u1*t^2; v2+2*u1*t2-u1*t];
y = yy;


eqnsd = [yy(1) == X1_T, yy(2) == X2_T];
sol   = solve(eqnsd,[t2,t]);
d2    = expand(sol.t2);
tau   = simplify(sol.t2)     % tau = d1
T     = simplify(sol.t)

tau1 = log((u1 + c*xa_0 - c^(1/2)*va_0)/(u1 + u2) + ((u2 - X1_T*c + X2_T*c^(1/2))*(2*u1*u2 + X2_T^2*c + c*va_0^2 + c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0))/(2*(u1 + u2)*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)
tau2 = log((u1 + c*xa_0 - c^(1/2)*va_0)/(u1 + u2) + ((u2 - X1_T*c + X2_T*c^(1/2))*(2*u1*u2 + X2_T^2*c + c*va_0^2 - c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0))/(2*(u1 + u2)*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)

T1   = log((2*u1*u2 + X2_T^2*c + c*va_0^2 + c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0)/(2*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)
T2   = log((2*u1*u2 + X2_T^2*c + c*va_0^2 - c^(1/2)*(X1_T^4*c^3 - 4*X1_T^3*c^2*u2 - 2*X1_T^2*X2_T^2*c^2 - 2*X1_T^2*c^3*xa_0^2 - 4*X1_T^2*c^2*u1*xa_0 + 2*X1_T^2*c^2*va_0^2 - 4*X1_T^2*c*u1^2 - 4*X1_T^2*c*u1*u2 + 4*X1_T^2*c*u2^2 + 4*X1_T*X2_T^2*c*u2 + 4*X1_T*c^2*u2*xa_0^2 + 8*X1_T*c*u1*u2*xa_0 - 4*X1_T*c*u2*va_0^2 + 8*X1_T*u1^2*u2 + 8*X1_T*u1*u2^2 + X2_T^4*c + 2*X2_T^2*c^2*xa_0^2 + 4*X2_T^2*c*u1*xa_0 - 2*X2_T^2*c*va_0^2 + 4*X2_T^2*u1^2 + 4*X2_T^2*u1*u2 + c^3*xa_0^4 + 4*c^2*u1*xa_0^3 - 2*c^2*va_0^2*xa_0^2 + 4*c*u1^2*xa_0^2 - 4*c*u1*u2*xa_0^2 - 4*c*u1*va_0^2*xa_0 - 4*c*u2^2*xa_0^2 + c*va_0^4 - 8*u1^2*u2*xa_0 - 8*u1*u2^2*xa_0 + 4*u1*u2*va_0^2 + 4*u2^2*va_0^2)^(1/2) - X1_T^2*c^2 - c^2*xa_0^2 + 2*X1_T*c*u2 - 2*c*u1*xa_0)/(2*(u1*u2 + c^(1/2)*u2*va_0 - X1_T*c*u1 + X2_T*c*va_0 + c*u2*xa_0 + X2_T*c^(1/2)*u1 - X1_T*c^(3/2)*va_0 - X1_T*c^2*xa_0 + X2_T*c^(3/2)*xa_0)))/c^(1/2)


%Knowns
X1_T_Val = .5;
X2_T_Val =  0;

va_0_Val = 0.4505
xa_0_Val =-0.0693


beta     = .31
alpha    = 9.81
c_Val    = -sqrt(.1)
%Input Knowns
d2    = subs(d2, u1,   beta);
d2    = subs(d2, u2,   alpha);
d2    = subs(d2, X1_T, X1_T_Val);
d2    = subs(d2, X2_T, X2_T_Val);
d2    = subs(d2, xa_0, xa_0_Val);
d2    = subs(d2, va_0, va_0_Val);
d2    = subs(d2, c, c_Val);
d2    = double(max(d2));

d1    = expand(sol.t);

d1    = subs(d1, u1,   beta);
d1    = subs(d1, u2,   alpha);
d1    = subs(d1, X1_T, X1_T_Val);
d1    = subs(d1, X2_T, X2_T_Val);
d1    = subs(d1, xa_0, xa_0_Val);
d1    = subs(d1, c, c_Val);
d1    = subs(d1, va_0, va_0_Val);

d1 = double(max(d1)) % T = d1

I = eye(2);

tau  = real(d2)
T    = real(d1)
Tend = T;  
dt  = .001;
t   =  0:dt:Tend;
u_Array = zeros(length(t),1);
for i = 1:length(t)
    ti = t(i);
    if ti < tau
        u_Array(i) = beta;
    else
        u_Array(i) = -alpha;
    end
end

X = zeros([length(t),2]); % Position and Velocity
X(1,:) = [xa_0_Val,va_0_Val];
A = [0     1; 
     c_Val 0]
for k = 1:length(t)-1
    X(k+1,:) = (I+A*dt)*X(k,:)' + B*dt*u_Array(k);
end

figure(1)
subplot(311)
plot(t,u_Array)
title('Control')
xlabel("Time")
ylabel("Control (N)")
xlim([0 Tend])
ylim([-alpha-1 beta+1])


subplot(312)
plot(t,X(1:length(X),1))
title('Position Vs Time')
xlim([0 Tend])
ylim([min(X(:,1)) max(X(:,1))])
xlabel('time')
ylabel("Position (m)")

subplot(313)
plot(t,X(1:length(X),2))
title('Velocity vs Time')
xlabel("Time")
ylabel("Vel (m/s)")
xlim([0 Tend])
ylim([min(X(:,2)) 1])

