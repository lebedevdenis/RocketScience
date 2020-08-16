function pwl = points2pwl(s,y)
% point2pwl Find a Piecewise Linear Function through 3 points
% function [a,b,s] = point2pwl(x,y)
% x and y are vectors containing the coordinates of the 3 points
% a are slopes
% b are offsets
% s are the split points including the start and end points

n=length(s); %number of split points

% Initialise the slope and intercept of each section to zero
a = zeros(1,n-1);    % slopes
b = zeros(1,n-1);    % offsets

par=zeros(2,n-1); %parameter storage

% Find the slope and intercept of each section
for i=1:n-1 %n points, n-1 sections
    par(:,i)=[s(i:i+1),ones(2,1)]\y(i:i+1);
end
a(:)=par(1,:);
b(:)=par(2,:);


pwl.a = a;
pwl.b = b;
pwl.s = s;
    
