function [H,V] = lander_noThrust()
% Lander version with thrust = 0 to obtain trajectory in H-V space without
% fuel use for initial fall before break control takes over
% Lander simulator
% Simulate a rocket assisted landing on a planet or a moon.
% version 9 piecewise linear model using selected values

%Initialise the model variables
% Initialise Vehicle structure
state.fixedmass = 500;          % kg  Mass with no fuel
state.fuelmass = 1500;          % kg
state.g = 9.81;                 % Acceleration due to gravity m s^-2
state.v = -300;                 % Velocity m /s
state.h = 3000;                 % Height m
state.t = 0;                    % Time s
state.dt = 0.1;                 % Step size s
state.burnrate = 0.001;         % fuel burnt per Newton thrust per second
state.dragconst = 1/3;          % the drag constant
state.a=-state.g+state.dragconst*(state.v)^2/(state.fixedmass+state.fuelmass); %previous acceleration not defined, m/s/s
%Initialize Controller structure
%ctrl.gain = 1000;               % The controller gain
%INITIALISED AS OPTIMISATION VARIABLE

% The three points used to define a piecewise linear function

%Initialise vectors to store the data to plot
n = 10000;              % The maximum number of data points
A = zeros(1,n);         % Vector to store the values of a.
V = A;                  % Vector to store the values of v.
H = A;                  % Vector to store the values of h.
T = A;                  % Vector to store the values of T.
M = A;                  % Vector to store the Mass

k = 0;

% While the height is greater than zero
while(state.h>0  && k<n)
    % Store the data to plot later
    k=k+1;
    A(k) = state.a;
    V(k) = state.v;
    H(k) = state.h;
    T(k) = state.t;
    M(k) = state.fuelmass;
    
    % Get the thrust from the controller
    thrust = 0;
    
    
    % Run the simulator to predict what will happen over the next time step
    state = Simulator(thrust,state);
    
end
%PlotGraphs(M,A,V,H,T)
V=V(1:k);
H=H(1:k);
end

