function PlotGraphs(M,A,V,H,T)

%Plot the lander trajectory

% height plot
subplot (4 ,2 ,2);
plot (T,H,'-r'); 
ylabel (' height (m) ');
xlabel (' time (s) ');
title (' Multiple piecewise linear function model ');
axis([0,30,0,3000])
grid on

% velocity plot
subplot (4 ,2 ,4);
plot (T,V,'-b'); 
ylabel (' velocity (ms^{-1}) ');
xlabel (' time (s) ');

%display landing velocity
s = sprintf('%0.2f m/s', V(end));
text(T(end),V(end),s);
axis([0,30,-300,0])
grid on

% acceleration plot
subplot (4 ,2 ,6);
plot (T,A,'-g'); 
ylabel (' acceleration (ms^{-2}) ');
xlabel (' time (s) ');

%display the max acceleration
[amax,I] = max(A);
amaxtime = T(I);
s = sprintf('  max %0.2f',amax);
hold on
plot(amaxtime,amax,'rx');
text(amaxtime,amax,s);
axis([0,30,0,6*9.81])
grid on
hold off

% Mass plot
subplot (4 ,2 ,8);
plot (T,M,'-m'); 
hold on
ylabel (' fuel mass (kg) ');
xlabel (' time (s) ');

%display final mass
s = sprintf('%0.2f kg', M(end));
text(T(end),M(end),s);
grid on
axis([0,30,500,1500])
hold off

pause(0.001)
