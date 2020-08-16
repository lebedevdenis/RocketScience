function y = pwlcalc6(pwl,x)
%evaluates pwl at x
a = pwl.a; %vector of slopes
b = pwl.b; %vector of offsets
s = pwl.s; %vector of split points

k=find(s < x,1,'last'); %find last split point leq x, i.e. index of pwl segment
y=a(k)*x+b(k);
if isempty(y)
    y=0; %happens when evaluate at h=0
end
end