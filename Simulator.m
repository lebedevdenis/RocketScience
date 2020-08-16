function newstate = Simulator(thrust,state)
% One step of the Lander Simulator
% Predict the state of the lander after the next time step

% Copy across any state variable that are not updated here.
newstate = state;       

if (state.fuelmass<=0)                % Have we run out of fuel
   % We have run out of fuel
   thrust = 0;                    % Without fuel there is no thrust          
   newstate.fuelmass = 0;            % If the mass is negative, set to zero
else
  % We have not run out of fuel
  % Calculate the fuel used and subtract from the fuelmass.
  newstate.fuelmass = state.fuelmass - abs(thrust) .* state.burnrate .*state.dt;  
end

%Calculate the Acceleration
newstate.a = getAcceleration(thrust,state);

%4. Use Euler's method to find the position at the next step
newstate.h = state.h + state.dt * state.v;

%5. Use Euler's method to find the velocity at the next step
newstate.v = state.v + state.dt * state.a;

% Increase the time by one time step
newstate.t = state.t + state.dt;

