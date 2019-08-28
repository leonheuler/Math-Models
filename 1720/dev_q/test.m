clc







r1 = [0 0 1]';


%% simulation config
figure;

axis([-1,1,-1,1,-1,1]);
% axis auto;
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis ij

back = animatedline;
left = animatedline;
right = animatedline;



simulationRate = 20; % Hz
simulationTime = 5; % sec
simulationLoops = simulationRate*simulationTime;

q = 0;

simulationLoop = robotics.Rate(simulationRate);
reset(simulationLoop);
for i = 1:simulationLoops
    
    q = q + pi/24;

    r1 = Rx(q)*r1;
    
    
    
    addpoints(back, 0, 0, 0);
    addpoints(back, r1(1), r1(2), r1(3));
    
	waitfor(simulationLoop);
end
disp(statistics(simulationLoop));
disp(simulationLoop);


function ret = Rx(q)
    ret = axang2rotm([1 0 0 q])
end
function ret = Ry(q)
    ret = axang2rotm([0 1 0 q]);
end
function ret = Rz(q)
    ret = axang2rotm([0 0 1 q]);
end
