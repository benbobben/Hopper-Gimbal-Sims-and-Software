%% To Initialize 1DoF Model
% Purpose is to just control a fixed system that can rotate about one axis 

clc
clear

addpath("Transformation Functions")

%% Initial Attitude
Euler_Initial = [0 0 0];
q_0           = Euler_To_Quaternion_ScalarFirst(Euler_Initial)';

%% Desired Attitude
desire_Angle  = deg2rad(45.0);
Euler_Desired =   [desire_Angle 0 0];
q_desired     =   Euler_To_Quaternion_ScalarFirst(Euler_Desired)';%Scalar First

desiredBodyRates = [0 0 0];

%% Initial Body rates
pqr_0            = [0 0 0]';

%% Applied Moments
Moments       = [0 0 0];

% Moments =           [0 1 2 3 4 10; 
%                      0 0 0 0 0 0]';
frictionCoeff = -1.92; 
%% Inertia
Inertia       = [ .10 0 0 
                  0 .10 0
                  0 0 .10 ];
%% Controller 
% ki = -.5;
% kp = -5;
% kd = -30;

ki = -.5;
kp = -10;
kd = -3;

%%
% controlInput  = [50,0,0]; %N
% momentArm     = .1 %m
err           = deg2rad(2);
% q_Err         = Euler_To_Quaternion_ScalarFirst(err)';%Scalar First

% Thrust_Moment = 10.2*momentArm; %Nm
Thrust_Moment = 14.2;
P_pulse       = .1; %Minimum Pulse length of time ; Thought of as Frequency
initialPos    = 0; 
PWM_Visualize_Gain = .01;
finalPos      = 10;%m
desiredVel    = 0;
delayLength   = 1; %Delay to allow for solver

%MPH Controller Only works for 1DOF in this program by forcing the it to
%change only the angle of interest
a = Euler_Desired(1); 
b = 0;
c = Euler_Initial(1);

alpha = Thrust_Moment; 
beta  = Thrust_Moment;

tau = -(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2))/(beta^2 + alpha*beta)
T   =  b/alpha - ((beta + alpha)*(alpha*b + b*beta - alpha*(((beta + alpha)*(- 2*beta*c + b^2 + 2*a*beta))/alpha)^(1/2)))/(alpha*(beta^2 + alpha*beta))

% sim('Hys_Control_1DoFSim.slx');
% sim('MPHControl_1DoFSim.slx');
% sim('PWMPIDControl_1DoFSim.slx');
