%---------------------------------------------Animation setup
fig1 = figure(1);
clf('reset')
fig1.Position = [400 200 500 500];
% set (gcf, 'WindowButtonMotionFcn', @mouseMove);

p = animatedline();
p.LineWidth = 2;
p.Marker = 'o';
p.MarkerSize = 6;



trajB = animatedline();


trajC = animatedline();
trajC.Color = 'red';
trajC_Marker = animatedline();
trajC_Marker.Color = 'red';
trajC_Marker.Marker = 'o';

axis equal
grid on
drKwGround;


%---------------------------------------------Simulation config
global Time dt
Time = 2*6.28;
dt = 0.01;

Iterations =  (Time / dt);  

%---------------------------------------------Model properities
global L1 L2

L1 = 0.425;
L2 = 0.47;

%---------------------------------------------Variables to save simulation data

global x_pos
global y_pos

global Data_T 
global Data_XK Data_YK
global Data_XA Data_YA
global Data_XC Data_YC
global Data_VX Data_VY
global Data_r Data_r_dt
global Data_q1
global Data_q2


Data_T = zeros(1,Iterations+1);
Data_XK = zeros(1,Iterations+1); Data_YK = zeros(1,Iterations+1);
Data_XA = zeros(1,Iterations+1); Data_YA = zeros(1,Iterations+1);
Data_XC = zeros(1,Iterations+1); Data_YC = zeros(1, Iterations+1);
Data_VX = zeros(1, Iterations+1); Data_VY = zeros(1, Iterations+1);
Data_r = zeros(1, Iterations+1); Data_r_dt = zeros(1, Iterations+1);
Data_q1 = zeros(1, Iterations+1);
Data_q2 = zeros(1, Iterations+1);



%---------------------------------------------Initial conditions
q1 = -0.001 * (pi / 180);        % bedro
q2 = 0 * (pi / 180);          % koleno
    

rK = [
    L1*sin(q1);
    -L1*cos(q1);
];

rA = [
     L1*sin(q1) + L2*sin(q1 - q2);
    -L1*cos(q1) - L2*cos(q1 - q2);
 ]; 

rC = rA;
rC_0 = rA;

err = rC - rA;
err_prev = err;
%--------------------------------------------------------trajectory
R = 0.05;
%--------------------------------------------------------System parameters
k = 0.5;         
%------------------------------------------------------------

%---------------------------------------------Simulation Task1 START 
tic
for it = 0:Iterations

    t = it*dt;
    
%     C = get (gca, 'CurrentPoint');
%     x_pos = C(1,1);
%     y_pos = C(1,2);
    
    rC(1) = rC_0(1) + R*sin(t);
    rC(2) = rC_0(2) + 1*R + R*cos(t);


        
    if norm(rC) > L1 + L2      
        rC = 0.9999*(L1+L2)*(rC/norm(rC));
    end
    
    err = rC-rA;
    r_dt = (err-r_prev)/dt;
    
    rA = rA + k*err;
   
    if norm(rA) > L1 + L2    
        rA = 0.9999*(L1+L2)*(rA/norm(rA));
    end
    
    
%----------------------------Inverse Kinematics
    
    q2 = acos( (norm(rA)^2 - L1^2 - L2^2) / (2*L1*L2) );
    q1 = -atan(rA(1)/rA(2)) + atan( L2*sin(q2)/(L1 + L2*cos(q2)));


    
%----------------------------Forward Kinematics (Animation purposes)
    
    rK = [
        L1*sin(q1);
        -L1*cos(q1);
    ];
        

%     rA = [
%          L1*sin(q1) + L2*sin(q2 - q1);
%         -L1*cos(q1) - L2*cos(q2 - q1);
%      ]; 

%----------------------------------------------------------Debug    
%     fprintf("L2 = %f\n", norm(rK-rA));
%---------------------------------------------------------Save Derivatives
    r_prev = err;
%----------------------------Save data
    Data_T(it+1) = t;
    Data_XK(it+1) = rK(1); Data_YK(it+1) = rK(2);
    Data_XA(it+1) = rA(1); Data_YA(it+1) = rA(2);
    Data_XC(it+1) = rC(1); Data_YC(it+1) = rC(2);
    
    Data_r(it+1) = norm(err);
    Data_r_dt(it+1) = norm(r_dt);
    
    Data_q1(it+1) = q1;
    Data_q2(it+1) = q2;
    
%----------------------------Animation
    addpoints(p, 0, 0);
    addpoints(p, rK(1), rK(2));
    addpoints(p, rA(1), rA(2));
    addpoints(trajB, rA(1), rA(2));
    addpoints(trajC, rC(1), rC(2));
    addpoints(trajC_Marker, rC(1), rC(2));
   
    drawnow;    % comment out to disable animation
    clearpoints(p);
    clearpoints(trajC_Marker);
    
end
%---------------------------------------------Simulation Task1 END
toc


fprintf("q1 = %f, q2 = %f\n", q1*(180/pi), q2*(180/pi));


drawPositionMap(1);

% ShowPlot('P');
% ShowPlot('V');
% ShowPlot('A');

ShowPlot('Q');
ShowPlot('W');
ShowPlot('dW');

% ShowPlot('E');
% ShowPlot('sigmaE');

%---------------------------------------------------Function Definitions

function drawPositionMap(n)
    
    global Data_XK Data_YK
    global Data_XA Data_YA
    
    len = length(Data_XK);
    
    pm = animatedline();
    pm.LineWidth = 2;
    pm.Marker = 'o';
    pm.MarkerSize = 6;
    
    i = 1;
    while i < len
        addpoints(pm, 0, 0);
        addpoints(pm, Data_XK(i), Data_YK(i));
        addpoints(pm, Data_XA(i), Data_YA(i));
        addpoints(pm, Data_XK(i), Data_YK(i));
        addpoints(pm, 0, 0);
        drawnow;
        i = i + round(len/n);
    end
    
    addpoints(pm, 0, 0);
    addpoints(pm, Data_XK(len), Data_YK(len));
    addpoints(pm, Data_XA(len), Data_YA(len));
    addpoints(pm, Data_XK(len), Data_YK(len));
    addpoints(pm, 0, 0);
    drawnow;
    
end


function drKwGround()
    sz = 0.04;   
    h = animatedline();
    h.LineWidth = 1.5;
    addpoints(h, 0, 0);
    addpoints(h, -sz*sin(30), -1.4*sz*cos(60));
    addpoints(h, sz*sin(30), -1.4*sz*cos(60));
    addpoints(h, 0, 0);
    
end

function ret = get_poli5_val(p,t)

    ret = p(1)*t^5 + p(2)*t^4 + p(3)*t^3 + p(4)*t^2 + p(5)*t;
    
end

function ret = get_poli6_val(p,t)

    ret = p(1)*t^6 + p(2)*t^5 + p(3)*t^4 + p(4)*t^3 + p(5)*t^2 + p(6)*t;
    
end

