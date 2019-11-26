%% Model config
global m1 m2 m3 L1 L2 L3 g Lh Lt Ch Chj Ct

m = 70;
H = 1.7;
L2 = 0.2450*H;
L1 = 0.2460*H;
L3 = H - (L1 + L2);

Lh = 0.75*L2;
Lt = 0.8*L3;

m1 = 0.0475*m;
m2 = 0.105*m;
m3 = 0.551*m/2;

g = 9.81;

Ch = 10000;
Ct = 10000;
Chj = 10000;

%% Simulation config
q0 = [pi/2-0.001 pi/2+0.0001 pi/2-0.001 0 0 0 ].'; % initial conditions
tbeg = 0;
tend = pi/2;
dt = 0.01;
tspan = tbeg:dt:tend; 
maxit = ceil((tend - tbeg) / dt);

%% Simdata config
fig2 = figure(2);
clf('reset');
global tau1p tau2p tau3p
tau1p = animatedline('Color','r');
tau2p = animatedline('Color','g');
tau3p = animatedline('Color','b');

fig3 = figure(3);
clf('reset');
global hq1p hq2p hq3p eq1p eq2p eq3p
subplot(3,1,1);
hq1p = animatedline('Color','r'); hold on;
eq1p = animatedline('Color','b');
subplot(3,1,2);
hq2p = animatedline('Color','r'); hold on;
eq2p = animatedline('Color','b');
subplot(3,1,3);
hq3p = animatedline('Color','r'); hold on;
eq3p = animatedline('Color','b');

fig4 = figure(4);
clf('reset');
global Hp HJp Tp gammaHp gammaHJp gammaTp
subplot(3,2,1);
Hp =  animatedline();
subplot(3,2,2);
gammaHp = animatedline();
subplot(3,2,3);
HJp = animatedline();
subplot(3,2,4);
gammaHJp = animatedline();
subplot(3,2,5);
Tp =  animatedline();
subplot(3,2,6);
gammaTp =  animatedline();

%% Solver config
opts = odeset('OutputFcn',@odeplot); 

fig5 = figure(5);
clf(fig5,'reset');

%% Simlation 
tic;
[t,q] = ode23s(@sys, tspan, q0, opts);
% q = ode4(@sys,tspan,q0);
toc;

%% Collect data from plots 
[~, eq1] = getpoints(eq1p);
[~, eq2] = getpoints(eq2p);
[~, eq3] = getpoints(eq3p);

[~, hq1] = getpoints(hq1p);
[~, hq2] = getpoints(hq2p);
[~, hq3] = getpoints(hq3p);

eO2x = L1*cos(eq1);
eO2y = L1*sin(eq1);
eO3x = L1*cos(eq1) + L2*cos(eq2);
eO3y = L1*sin(eq1) + L2*sin(eq2);
eO4x = L1*cos(eq1) + L2*cos(eq2) + L3*cos(eq3);
eO4y = L1*sin(eq1) + L2*sin(eq2) + L3*sin(eq3);

hO2x = L1*cos(hq1);
hO2y = L1*sin(hq1);
hO3x = L1*cos(hq1) + L2*cos(hq2);
hO3y = L1*sin(hq1) + L2*sin(hq2);
hO4x = L1*cos(hq1) + L2*cos(hq2) + L3*cos(hq3);
hO4y = L1*sin(hq1) + L2*sin(hq2) + L3*sin(hq3);

%% Animation
fig6 = figure(6);
clf('reset');
human = animatedline('Marker','o','MarkerSize',15, 'MarkerFaceColor', 'black', ...
                    'LineWidth',16, 'Color', 'black');
                
exo =   animatedline('Marker','o','MarkerSize',4, ...
                    'LineWidth',4, 'Color', 'yellow');

tr = animatedline();
ecp = animatedline('Marker','o','MarkerSize',2,'Color','red');
ec1p = animatedline('Marker','o','MarkerSize',3,'Color','blue');
ec2p = animatedline('Marker','o','MarkerSize',3,'Color','blue');
ec3p = animatedline('Marker','o','MarkerSize',3,'Color','blue');

axis equal;


last = numel(eO2x);
for i=1:last-1

    addpoints(exo, 0, 0);
    addpoints(exo, eO2x(i), eO2y(i));
    addpoints(exo, eO3x(i), eO3y(i));
    addpoints(exo, eO4x(i), eO4y(i));
    addpoints(tr,eO4x(i), eO4y(i));
    
    addpoints(human, 0, 0);
    addpoints(human, hO2x(i), hO2y(i));
    addpoints(human, hO3x(i), hO3y(i));
    addpoints(human, hO4x(i), hO4y(i));
  
    eC1 = [eO2x(i)/2; 
           eO2y(i)/2];

    eC2 = [(eO2x(i)+eO3x(i))/2; 
           (eO2y(i)+eO3y(i))/2 ];

    eC3 = [(eO3x(i)+eO4x(i))/2; 
           (eO3y(i)+eO4y(i))/2 ];
           
    eC = (m1*eC1 + m2*eC2 + m3*eC3) / (m1 + m2 + m3);
    
    addpoints(ec1p, eC1(1), eC1(2));
    addpoints(ec2p, eC2(1), eC2(2));
    addpoints(ec3p, eC3(1), eC3(2));
    addpoints(ecp,  eC(1), eC(2));
    
    drawnow limitrate;
    
    clearpoints(ec1p);
    clearpoints(ec2p);
    clearpoints(ec3p);
    clearpoints(human);
    clearpoints(exo);
end

addpoints(exo, 0, 0);
addpoints(exo, eO2x(last), eO2y(last));
addpoints(exo, eO3x(last), eO3y(last));
addpoints(exo, eO4x(last), eO4y(last));
addpoints(tr,eO4x(last), eO4y(last));

addpoints(human, 0, 0);
addpoints(human, hO2x(last), hO2y(last));
addpoints(human, hO3x(last), hO3y(last));
addpoints(human, hO4x(last), hO4y(last));

addpoints(ec1p, eC1(1), eC1(2));
addpoints(ec2p, eC2(1), eC2(2));
addpoints(ec3p, eC3(1), eC3(2));
addpoints(ecp,  eC(1), eC(2));

%% ODE system in non-linear state-space representation
function dx = sys(t,x)

    global m1 m2 m3 L1 L2 L3 g Lh Lt Ch Chj Ct
    global tau1p tau2p tau3p
    global hq1p hq2p hq3p
    global eq1p eq2p eq3p
    global Hp HJp Tp
    global gammaHp gammaHJp gammaTp

    % State-space variables
    q = [x(1); x(2); x(3) ];
    qdot = [x(4); x(5); x(6)];

    % Human trajectory in joint space
    q_h = [ pi/2 - pi/8*(1-cos(t));   
           pi/2 + 2*pi/8*(1-cos(t));       
            pi/2 - pi/8*(1-cos(t)) ];
        
    % Human points of contact  
    hH = [ L1*cos(q_h(1))+Lh*cos(q_h(2));
             L1*sin(q_h(1))+Lh*sin(q_h(2)) ];
        
    hHJ = [ L1*cos(q_h(1))+L2*cos(q_h(2));
              L1*sin(q_h(1))+L2*sin(q_h(2)) ];   
          
    hT = [ L1*cos(q_h(1))+L2*cos(q_h(2)) + Lt*cos(q_h(3));
             L1*sin(q_h(1))+L2*sin(q_h(2)) + Lt*sin(q_h(3))];  
    
    % corresponding exo points
    eH = [ L1*cos(q(1))+Lh*cos(q(2));
           L1*sin(q(1))+Lh*sin(q(2)) ];
    
    eHJ = [ L1*cos(q(1))+L2*cos(q(2));
            L1*sin(q(1))+L2*sin(q(2)) ];
       
    eT = [ L1*cos(q(1))+L2*cos(q(2)) + Lt*cos(q(3));
           L1*sin(q(1))+L2*sin(q(2)) + Lt*sin(q(3))];   
       
    % Reaction forces
    H = Ch*(hH-eH);
    HJ = Chj*(hHJ - eHJ);
    T  = Ct*(hT - eT);
    
    % Their magnitudes
    Tnorm = norm(T);
    Hnorm = norm(H);
    HJnorm = norm(HJ);
    
    % Reaction forces absolute angles
    gammaT = acos(T(1)/Tnorm);
    gammaH = acos(H(1)/Hnorm);
    gammaHJ = acos(HJ(1)/HJnorm); 
    
    tau_d = [0 0 0].';
    
    tau_ext = [ Tnorm*L1*sin(gammaT+q_h(1)) + HJnorm*L1*sin(gammaHJ+q_h(1)) + Hnorm*L1*sin(gammaH+q_h(1));
                Tnorm*L2*sin(gammaT+q_h(2)) + HJnorm*L2*sin(gammaHJ+q_h(2)) + Hnorm*Lh*sin(gammaH+q_h(2));
                Tnorm*Lt*sin(gammaT+q_h(3)) ];
     tau_ext = - tau_ext;       
     
%     tau_ext = [ 0 0 0 ].';

    % Errors
%     e = q_d - q;
%     edot = qdot_d - qdot;
    
    % PID config
%     w = 100;
%     Kp = diag([w^2 w^2 w^2]);
%     Kv = diag([2*w 2*w 2*w]);

    % Control law
%     tau = M(q)*(Kp*e + Kv*edot) + N(q,qdot);         % CTC
%     tau = Kp*e + Kv*e_dot + vecG;               % PD-plus-Gravity
%     tau = G(q);
    tau = [0; 0; 0];
    
    % state-space representation
    dx = [ x(4); x(5); x(6); -M(q) \ N(q,qdot)] + [zeros(3,1); M(q) \ (tau + tau_ext)]+ [zeros(3,1); -M(q) \ tau_d];
%     dx = [ x(4); x(5); x(6); lsqminnorm(-M(q),N(q,qdot))] + [zeros(3,1); lsqminnorm(M(q),(tau + tau_ext))] + [zeros(3,1); lsqminnorm(-M(q),tau_d)];
    

    addpoints(hq1p, t, q_h(1));
    addpoints(hq2p, t, q_h(2));
    addpoints(hq3p, t, q_h(3));
 
    addpoints(eq1p, t, q(1));
    addpoints(eq2p, t, q(2));
    addpoints(eq3p, t, q(3));

    addpoints(Hp , t,  Hnorm);
    addpoints(HJp, t, HJnorm);
    addpoints(Tp , t,  Tnorm);
    
    addpoints(gammaTp,  t,  gammaT);
    addpoints(gammaHp,  t,  gammaH);
    addpoints(gammaHJp, t, gammaHJ);

    addpoints(tau1p, t, tau(1));
    addpoints(tau2p, t, tau(2));
    addpoints(tau3p, t, tau(3));

    drawnow limitrate;
    
end



function ret = M(q)
    
    global m1 m2 m3 L1 L2 L3
    ret = [ m1*L1^2/3 + (m2+m3)*L1^2,           (m2/2+m3)*L1*L2*cos(q(1)-q(2)),     m3*L1*L3/2*cos(q(1)-q(3));
           (m2+2*m3)*L1*L2/2*cos(q(2)-q(1)),    m2*L2^2/3 + m3*L2^2,               m3*L2*L3/2*cos(q(2)-q(3))
            m3*L1*L3/2*cos(q(3)-q(1)),           m3*L2*L3/2*cos(q(3)-q(2)),         m3*L3^2/3   ];

end

function ret = V(q,qdot)

    global m1 m2 m3 L1 L2 L3
    ret  = [ qdot(2)^2*(m2/2+m3)*L1*L2*sin(q(1)-q(2)) + qdot(3)^2*m3*L1*L3/2*sin(q(1)-q(3));
             qdot(1)^2*(m2+2*m3)*L1*L2*sin(q(2)-q(1)) + qdot(3)^2*m3*L2*L3/2*sin(q(2)-q(3));
             qdot(1)^2*m3*L1*L3/2*sin(q(3)-q(1)) + qdot(2)^2*m3*L2*L3/2*sin(q(3)-q(2)) ];

end
    
    
function ret = G(q)
    
    global m1 m2 m3 L1 L2 L3 g
    ret = [ (m3+m2+m1/2)*g*L1*cos(q(1));
            (m3+m2/2)*g*L2*cos(q(2));
            m3*g*L3/2*cos(q(3)) ];

end
    

function ret = N(q,qdot)

    ret = V(q,qdot) + G(q) + F(qdot);

end
    
    
    