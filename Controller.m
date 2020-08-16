function thrust = Controller(state,ctrl)

% The Lander Controller 
% Calulate the thurst required to obtain a desired velocity.

% Calculate the Lander mass
m = state.fixedmass + state.fuelmass;

%Acceleration due to gravity
g = state.g;


% The target velocity
if state.h > ctrl.hBreak
    vd = polyval(ctrl.free,state.h);
else
    vd = pwlcalc6(ctrl.pwl,state.h);
end
% The controller gain
k = ctrl.gain;

% Calculate the error
Error = vd - state.v;

%The controller
%Proportional control plus weight and drag. No negative thrust.
thrust = max(0,k .* Error + m .* g + state.dragconst*abs(state.v)*state.v);     

end

