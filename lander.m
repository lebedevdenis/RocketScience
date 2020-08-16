function obj = lander(x,boolPlot,p_freeFall)
% Function version of below script definition. For minimisation.
% Lander simulator
% Simulate a rocket assisted landing on a planet or a moon.
% version 9 piecewise linear model using selected values
% now also includes free-fall model and selects height from which to start
% braking

%x1:x_n    decision velocities
%x_n+1     height to start breaking
%x_end     gain
n=length(x-2); %-2 for hBreak and gain
ctrl.hBreak=x(end-1);
vBreak=polyval(p_freeFall,ctrl.hBreak);
vSplit=[x(1:end-2);vBreak];
hSplit=linspace(0,ctrl.hBreak,n-1)';
ctrl.gain=x(end);
ctrl.free=p_freeFall;
% The n-1 points used to define a piecewise linear function
ctrl.pwl=points2pwl(hSplit,vSplit);

if boolPlot
    hold off
    % plot to check
    h=1:3000;
    v=zeros(length(h),1);
    for i =1:length(h)
        if h(i)>ctrl.hBreak
            v(i)=polyval(ctrl.free,h(i));
        else
            v(i)=pwlcalc6(ctrl.pwl,h(i));
        end
    end
    subplot(4,2,1:2:7)
    plot(h,v)
    hold on
    plot(h,polyval(p_freeFall,h),'g--')
    axis([0,3000,-300,0])
    grid on
    title("Velocity against height")
    xlabel("height (m)")
    ylabel("velocity (ms^{-1})")
    text(2500,-150,sprintf("iteration %i", n-3))
    %pause(0.001)
    hold off
end

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
    thrust = Controller(state,ctrl);
        
    % Run the simulator to predict what will happen over the next time step
    state = Simulator(thrust,state);
    
end

hFinal=H(k);
vFinal=V(k);
aMax=max(A);

%objective value, penalise maximum acceleration and final velocity
obj=-M(k)+0.1*aMax-10*vFinal;
if M(k) > 0
    hFinal=0;
end
if -sqrt(state.g*hFinal)+vFinal < -2 || aMax > 6*0.9*9.81
    obj = inf;%include constraints implicitly
    %Optional outputs for debugging
    %sprintf("hFinal: %f", hFinal)
    %sprintf("vFinal: %f", vFinal)
    %sprintf("aMax: %f", aMax)
    %sprintf("minusFuelFinal: %f", minusFuelFinal)
end

if boolPlot
    M=M(1:k);
    A=A(1:k);
    V=V(1:k);
    H=H(1:k);
    T=T(1:k);
    
    hold on
    %plot actual h-v trajectory
    plot(H,V,'r')
    legend("desired velocity","freefall velocity","actual velocity")
    grid on
    hold off
    %plot simulated telemetry data
    PlotGraphs(M,A,V,H,T)
end
end