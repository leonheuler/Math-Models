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
drawGround;


%---------------------------------------------Simulation config
global Time dt
Time = 2*6.28;
dt = 0.001;

Iterations =  (Time / dt);  

%---------------------------------------------Model properities
global L1 L2

L1 = 0.425;
L2 = 0.47;

%---------------------------------------------Variables to save simulation data

global x_pos
global y_pos

global Data_T 
global Data_XA Data_YA
global Data_XB Data_YB
global Data_XC Data_YC
global Data_VX Data_VY
global Data_r Data_r_dt
global Data_q1
global Data_q2


Data_T = zeros(1,Iterations+1);
Data_XA = zeros(1,Iterations+1); Data_YA = zeros(1,Iterations+1);
Data_XB = zeros(1,Iterations+1); Data_YB = zeros(1,Iterations+1);
Data_XC = zeros(1,Iterations+1); Data_YC = zeros(1, Iterations+1);
Data_VX = zeros(1, Iterations+1); Data_VY = zeros(1, Iterations+1);
Data_r = zeros(1, Iterations+1); Data_r_dt = zeros(1, Iterations+1);
Data_q1 = zeros(1, Iterations+1);
Data_q2 = zeros(1, Iterations+1);



%---------------------------------------------Initial conditions
q1 = 0.001 * (pi / 180);        % bedro
q2 = 0 * (pi / 180);          % koleno


rA = [
    L1*sin(q1);
    -L1*cos(q2);
];

rB = [
     L1*sin(q1) + L2*sin(q1 - q2);
    -L1*cos(q1) - L2*cos(q1 - q2);
 ]; 

rC = rB;
rC_0 = rB;

% residual
r = (rC-rB); 
r_prev = r; 

%---------------------------------------------C trajectory parameters

% x
v0_x = 0; v1_x = 0;
a0_x = 0; a1_x = 0;
% y
v0_y = 0; v1_y = 0;
a0_y = 0; a1_y = 0;

L = 0.5;
H = 0.05;

p5x = s_poli5(Time, L, v0_x, v1_x, a0_x, a1_x);
p6y = s_poli6(Time, H, v0_y, v1_y, a0_y, a1_y);

%--------------------------------------------------------System parameters
k = 0.2;         


%---------------------------------------------Simulation Task1 START 
tic
for it = 0:Iterations
   
%     C = get (gca, 'CurrentPoint');
%     x_pos = C(1,1);
%     y_pos = C(1,2);
    
    t = it*dt;
%      w = 10;
    radius = 0.05;
%     rC(1) = rD_0(1) + get_poli5_val(p5x, t); 
%     rC(2) = rD_0(2) + get_poli6_val(p6y, t);
%       rC(1) = rC_0(1)*cos(-w*t);
%       rC(2) = rC_0(1)*sin(-w*t) + rC_0(2);
    rC(1) = radius * cos(t - pi/2);
    rC(2) = -(L1+L2) + radius + radius * sin(t - pi/2);
%     rC(1) = x_pos;
%     rC(2) = y_pos;
    
    if norm(rC) > L1 + L2      
        rC = 1*(L1+L2)*(rC/norm(rC));
    end

    r = rC-rB;
    r_dt = (r-r_prev)/dt;

    rB = rB + k*r;
   
    if norm(rB) > L1 + L2    
        rB = 1*(L1+L2)*(rB/norm(rB));
    end
    
    
%----------------------------Inverse Kinematics
    
    q2 = acos( (norm(rB)^2 - L1^2 - L2^2) / (2*L1*L2) );
    q1 = -atan(rB(1)/rB(2)) + atan( L2*sin(q2)/(L1 + L2*cos(q2)));


    
%----------------------------Forward Kinematics (Animation purposes)
    
    rA = [
        L1*sin(q1);
        -L1*cos(q1);
    ];
        

%     rB = [
%          L1*sin(q1) + L2*sin(q2 - q1);
%         -L1*cos(q1) - L2*cos(q2 - q1);
%      ]; 

%----------------------------------------------------------Debug    
%     fprintf("L2 = %f\n", norm(rA-rB));
%---------------------------------------------------------Save Derivatives
    r_prev = r;
%----------------------------Save data
    Data_T(it+1) = t;
    Data_XA(it+1) = rA(1); Data_YA(it+1) = rA(2);
    Data_XB(it+1) = rB(1); Data_YB(it+1) = rB(2);
    Data_XC(it+1) = rC(1); Data_YC(it+1) = rC(2);
    
    Data_r(it+1) = norm(r);
    Data_r_dt(it+1) = norm(r_dt);
    
    Data_q1(it+1) = q1;
    Data_q2(it+1) = q2;
    
%----------------------------Animation
    addpoints(p, 0, 0);
    addpoints(p, rA(1), rA(2));
    addpoints(p, rB(1), rB(2));
    addpoints(trajB, rB(1), rB(2));
    addpoints(trajC, rC(1), rC(2));
    addpoints(trajC_Marker, rC(1), rC(2));
   
%     drawnow;    % comment out to disable animation
    clearpoints(p);
    clearpoints(trajC_Marker);
    
end
%---------------------------------------------Simulation Task1 END
toc


fprintf("q1 = %f, q2 = %f\n", q1*(180/pi), q2*(180/pi));


drawPositionMap(1);

ShowPlot('P');
ShowPlot('V');
% ShowPlot('A');

ShowPlot('Q');
ShowPlot('W');
ShowPlot('dW');

% ShowPlot('E');
% ShowPlot('sigmaE');

%---------------------------------------------------Function Definitions

function drawPositionMap(n)
    
    global Data_XA Data_YA
    global Data_XB Data_YB
    
    len = length(Data_XA);
    
    pm = animatedline();
    pm.LineWidth = 2;
    pm.Marker = 'o';
    pm.MarkerSize = 6;
    
    i = 1;
    while i < len
        addpoints(pm, 0, 0);
        addpoints(pm, Data_XA(i), Data_YA(i));
        addpoints(pm, Data_XB(i), Data_YB(i));
        addpoints(pm, Data_XA(i), Data_YA(i));
        addpoints(pm, 0, 0);
        drawnow;
        i = i + round(len/n);
    end
    
    addpoints(pm, 0, 0);
    addpoints(pm, Data_XA(len), Data_YA(len));
    addpoints(pm, Data_XB(len), Data_YB(len));
    addpoints(pm, Data_XA(len), Data_YA(len));
    addpoints(pm, 0, 0);
    drawnow;
    
end


function drawGround()
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

