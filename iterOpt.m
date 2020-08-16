%iteropt.m
%Iteratively adds (piecewise linear) pwl segments to optimise trajectory

%% Optimisation parameters
nSplit = 3; %number of initial split points (including start and end)
nSplitMax = 32; %number of final split points
hBreak = 2500; %initial height when to start breaking [m]
hSplit = linspace(0,hBreak,nSplit)'; %split point heights, evenly distributed [m]
vd.Final = -1; %final desired velocity [m/s]
vd.Initial = -300; %initial velocity [m/s]

%simulate rocket crash without thrust to estimate free-fall frajectory
[H_noThrust, V_noThrust] = lander_noThrust();
p_freeFall = polyfit(H_noThrust,V_noThrust,3);

%velocity at hBreak
vBreak=polyval(p_freeFall,hBreak);

%% Initialise optimisation variables
vSplit0 = vd.Final-(vd.Final-vBreak)/hBreak*hSplit(2:end-1); %initialise vSplit values using linear descent model
gain0 = 500; %scaled down by 10 to increase sensitivity of fmincon algorithm wrt gain, i.e. gain0 = z actually stands for a gain value of z*10
obj = NaN; %objective value
objStore=zeros(nSplitMax-2,1); %objective value storage

%% Initialise video
%% Initialize video
myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.Quality = 95;
myVideo.FrameRate = 2;
open(myVideo)

%% Optimise
set(gcf, 'Position', get(0, 'Screensize')); %full screen figure
for i=nSplit:nSplitMax
    fprintf("Iteration %i\n",i-2)
    
    %compute vSplit-values
    [y,obj]= funOpt(hSplit,vSplit0,gain0,vd,p_freeFall);
    [~] = lander([vd.Final;y(1:end-1);y(end)*10],true,p_freeFall); %true = plot
    
    %initialise next iteration with solution from this iteration
    gain0=y(end);
    hBreak=y(end-1);
    vBreak=polyval(p_freeFall,hBreak);
    hSplit=linspace(0,hBreak,nSplit)';
    objStore(i-2)=-obj;
    if i == nSplitMax
        break %don't add another split if it's the last iteration
    end
    
    %compute pwl
    pwl = points2pwl(hSplit,[vd.Final;y(1:end-2);vBreak]);
    %add pwl-segment
    nSplit = nSplit+1;
    hSplit = linspace(0,hBreak,nSplit)';
    vSplit0 = zeros(nSplit-2,1);
    for j=1:nSplit-2
        vSplit0(j) = pwlcalc6(pwl,hSplit(j+1));
    end
    
    %save plotted graph as a frame for the video
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)

%% Plot objective value
myV=[vd.Final;y(1:end-2);vBreak];
[~] = lander([myV(1:end-1);hBreak;10*gain0],true,p_freeFall); %true = plot
figure
plot(1:nSplitMax-2,objStore,'b','LineWidth',2)
grid on
title("Objective function value against iterations")
xlabel("Iteration")
ylabel("Objective function value")