%---------------------------------------------Animation setup
fig1 = figure(1);
clf('reset')
fig1.Position = [1400 500 500 500];
% set (gcf, 'WindowButtonMotionFcn', @mouseMove);

p = animatedline();
p.LineWidth = 2;
p.Marker = 'o';
p.MarkerSize = 6;

trajB = animatedline();

trajC = animatedline();
trajC_Marker = animatedline();
trajC_Marker.Marker = 'o';

trajD = animatedline();
trajD.Color = 'red';

trajD_Marker = animatedline();
trajD_Marker.Color = 'red';
trajD_Marker.Marker = 'o';

axis equal
grid on
drawGround;


%---------------------------------------------Simulation config
global Time dt
Time = 1;
dt = 0.001;

Iterations =  (Time / dt);  

%---------------------------------------------Model properities
global L1 L2 L3

L1 = 0.5;
L2 = 0.5;
L3 = 0.45;
%---------------------------------------------Variables to save simulation data

global x_pos
global y_pos

global Data_T 
global Data_XA Data_YA
global Data_XB Data_YB
global Data_XC Data_YC
global Data_XD Data_YD
global Data_VX Data_VY
global Data_V Data_dV
global Data_phi3
global Data_phi2


Data_T = zeros(1,Iterations+1);
Data_XA = zeros(1,Iterations+1); Data_YA = zeros(1,Iterations+1);
Data_XB = zeros(1,Iterations+1); Data_YB = zeros(1,Iterations+1);
Data_XC = zeros(1,Iterations+1); Data_YC = zeros(1, Iterations+1);
Data_XD = zeros(1,Iterations+1); Data_YD = zeros(1, Iterations+1);
Data_VX = zeros(1, Iterations+1); Data_VY = zeros(1, Iterations+1);
Data_V = zeros(1, Iterations+1); Data_dV = zeros(1, Iterations+1);
Data_phi3 = zeros(1, Iterations+1);
Data_phi2 = zeros(1, Iterations+1);



%---------------------------------------------Initial conditions
phi3 = -13 * (pi / 180);        % bedro
phi2 = 5 * (pi / 180);          % koleno
phi3_ = phi3;
phi2_ = phi2;

rA = [
    L1*sin(phi3);
    -L1*cos(phi3);
];

rC = [
     L1*sin(phi3) + L3*sin(phi3 - phi2);
    -L1*cos(phi3) - L3*cos(phi3 - phi2);
 ]; 

rB = [
     L1*sin(phi3) + L2*sin(phi3 - phi2);
    -L1*cos(phi3) - L2*cos(phi3 - phi2);
 ]; 


rC_p = rC;
rC_pp = rC;
    
rD = rC;
rD_0 = rD;

V = (rD-rC);
prevV = V;

%---------------------------------------------B trajectory parameters

% x
v0_x = 0; v1_x = 0;
a0_x = 0; a1_x = 0;
% y
v0_y = 0; v1_y = 0;
a0_y = 0.3; a1_y = 0;

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
     w = 10;
%     rD(1) = rD_0(1) + get_poli5_val(p5x, t); 
%     rD(2) = rD_0(2) + get_poli6_val(p6y, t);
      rD(1) = rD_0(1)*cos(-w*t);
      rD(2) = rD_0(1)*sin(-w*t) + rD_0(2);
%     rB(1) = x_pos;
%     rB(2) = y_pos;
    
    if norm(rD) > L1 + L3      
        rD = (L1+L3)*(rD/norm(rD));
    end

    V = k*(rD-rC);

    dV = (V-prevV)/dt;

    rC = rC + k*(rD-rC);
    
    
    if norm(rC) > L1 + L3     
        rC = (L1+L3)*(rC/norm(rC));
    end
    
    
%----------------------------Inverse Kinematics
    
    phi2 = acos( (norm(rC)^2 - L1^2 - L3^2) / (2*L1*L3) );
    phi3 = -atan(rC(1)/rC(2)) + atan( L3*sin(phi2)/(L1 + L3*cos(phi2)));


    
%----------------------------Forward Kinematics (Animation purposes)
    
    rA = [
        L1*sin(phi3);
        -L1*cos(phi3);
    ];
        

    rB = [
         L1*sin(phi3) + L2*sin(phi3 - phi2);
        -L1*cos(phi3) - L2*cos(phi3 - phi2);
     ]; 

%----------------------------------------------------------Debug    
%     fprintf("L2 = %f\n", norm(rA-rB));
%---------------------------------------------------------Save Derivatives
    prevV = V;
%----------------------------Save data
    Data_T(it+1) = t;
    Data_XA(it+1) = rA(1); Data_YA(it+1) = rA(2);
    Data_XB(it+1) = rB(1); Data_YB(it+1) = rB(2);
    Data_XC(it+1) = rC(1); Data_YC(it+1) = rC(2);
    Data_XD(it+1) = rD(1); Data_YD(it+1) = rD(2);
    
    Data_VX(it+1) = V(1); Data_VY(it+1) = V(2);
    
    Data_V(it+1) = norm(V);
    Data_dV(it+1) = norm(dV);
    
    Data_phi3(it+1) = phi3;
    Data_phi2(it+1) = phi2;
    
%----------------------------Animation
    addpoints(p, 0, 0);
    addpoints(p, rA(1), rA(2));
    addpoints(p, rC(1), rC(2));
    addpoints(p, rB(1), rB(2));
    addpoints(trajB, rB(1), rB(2));
    addpoints(trajC, rC(1), rC(2));
    addpoints(trajC_Marker, rC(1), rC(2));
    addpoints(trajD, rD(1), rD(2));
    addpoints(trajD_Marker, rD(1), rD(2));
    
   
    drawnow;    % comment out to disable animation
    clearpoints(p);
    clearpoints(trajC_Marker);
    clearpoints(trajD_Marker);
    
end
%---------------------------------------------Simulation Task1 END
toc


fprintf("phi3 = %f, phi2 = %f\n", phi3*(180/pi), phi2*(180/pi));


drawPositionMap(1);

ShowPlot('P');
% ShowPlot('V');
% ShowPlot('A');

ShowPlot('Q');
ShowPlot('W');
ShowPlot('dW');

ShowPlot('F');
ShowPlot('E');
% ShowPlot('sigmaE');

%---------------------------------------------------Function Definitions

function drawPositionMap(n)
    
    global Data_XA Data_YA
    global Data_XB Data_YB
    global Data_XC Data_YC
    
    len = length(Data_XA);
    
    pm = animatedline();
    pm.LineWidth = 2;
    pm.Marker = 'o';
    pm.MarkerSize = 6;
    
    i = 1;
    while i < len
        addpoints(pm, 0, 0);
        addpoints(pm, Data_XA(i), Data_YA(i));
        addpoints(pm, Data_XC(i), Data_YC(i));
        addpoints(pm, Data_XB(i), Data_YB(i));
        addpoints(pm, Data_XC(i), Data_YC(i));
        addpoints(pm, Data_XA(i), Data_YA(i));
        addpoints(pm, 0, 0);
        drawnow;
        i = i + round(len/n);
    end
    
    addpoints(pm, 0, 0);
    addpoints(pm, Data_XA(len), Data_YA(len));
    addpoints(pm, Data_XC(len), Data_YC(len));
    addpoints(pm, Data_XB(len), Data_YB(len));
    addpoints(pm, Data_XC(len), Data_YC(len));
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

