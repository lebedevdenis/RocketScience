
function a = getAcceleration(thrust,state)
%Calculate the acceleration of the lander

%Calculate the total mass of Lander and fuel kg
mass = state.fixedmass + state.fuelmass;    

%calculate the drag
drag = getDrag(state);

%Calculate the Force
force =  thrust + drag - mass*state.g;       % Total force in Newtons

%Use Newtons second law to calculate the acceleration
a = force /mass;                        % Acceleration m/s



