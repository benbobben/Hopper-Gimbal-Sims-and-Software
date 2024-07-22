%%% PID control of free mass using State Space Model
%%% Doble integrator with 2 inputs (rx, rdotx) and 2 outputs (x, \dot{x}) 
%%% with PID controller in state space form. 
%%%  Characteristic eq. of A matrix is 
%%%  s^3 + Ki s^2 + K s + Kd = 0

clear

%%%%

syms s

p = [-1 -2 -100];                          % desired poles
pol = expand((s-p(1))*(s-p(2))*(s-p(3))); % Expanding the polynomial 
pc = coeffs(pol);                         % extracting coeficients of polynomial

% A = [0 1; k/m b/m];
% B = [0; 1];

K     = double(pc(2));
Ki    = double(pc(1));
Kd    = double(pc(3));

A   = [ 0  1  0 0 ; 
       -K -Kd Ki 1; 
       -1  0  0 0 
       0 0 0 1];

B   = [0 0; 
       K Kd; 
       1 0;
       0 0 ];

rx  = 0.5;  % Desired Position
rdx = 0;    % Desired Speed

sys = ss(A,B,[1 0 0 0;0 1 0 0 ],zeros(2))

T = linspace(0,10,1000);
U = [[rx*ones(1,length(T)/2)  0.5*rx*ones(1,length(T)/2)];  
      rdx*ones(1,length(T))]';

y = lsim(sys,U,T);

% Position 
subplot(2,1,1)
plot(T,y(:,1))
title("Position")

% Velocity 
subplot(2,1,2)
plot(T,y(:,2))
title("Velocity")

%%% Checks and balances
H = tf(sys);

eig(A)


