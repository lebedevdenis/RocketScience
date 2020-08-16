function drag = getDrag(state)

% Calculate the drag acting on the Lander

drag = - state.dragconst .* abs(state.v) .* state.v;