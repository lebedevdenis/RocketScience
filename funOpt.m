function [y,obj] = funOpt(hSplit,vSplit0,gain0,vd,p_freeFall)
%FUNOPT computes optimal pwl h-v landing trajectory of P5 Computing rocket

%% Set-up
%find height and velocity at which breaking is applied for the first time
hBreak=hSplit(end);
vBreak=polyval(p_freeFall,hBreak);

x0 = [vSplit0;hBreak;gain0]; %initial guess
options = optimset('Display','none');

%% Optimisation
fun = @(x)lander([vd.Final;x(1:end-2);x(end-1);10*x(end)],false,p_freeFall); %false=don't plot
[y,obj,exitflag] = fmincon(fun,x0,[],[],[],[],...
    [polyval(p_freeFall,hSplit(2:end-1));hBreak;100],...
    [vd.Final-(vd.Final-vBreak)/hBreak*hSplit(2:end-1);hBreak;1000],[],...
    options);

%% Optional prints for debugging
%sprintf("obj: %.2f", obj)
%sprintf("exitflag: %i", exitflag) 
end

