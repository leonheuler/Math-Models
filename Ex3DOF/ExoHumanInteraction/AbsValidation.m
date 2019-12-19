%% Model config
global hm1 hm2 hm3 m1 m2 m3 L1 L2 L3 g Lh Lt 
global Ch Chj Ct Bh Bhj Bt

m = 70;
H = 1.8;
L2 = 0.2450*H;
L1 = 0.2460*H;
L3 = H - (L1 + L2);

Lh = 0.3*L2;
Lt = 0.8*L3;

hm1 = 0.0475*m;
hm2 = 0.105*m;
hm3 = 0.551*m/2;

m1 = 8;
m2 = 8;
m3 = 8;

g = 9.81;

var1 = 1000;
Ch =  var1; Bh = 0;
Ct =  var1; Bt = 0;
Chj = var1; Bhj = 0;

%% Simulation config
% q0 = [pi/2 pi/2 pi/2 0 0 0 ].'; 
q0 = [pi/4 pi/2+pi/4 pi/4 0 0 0 ].'; 
tbeg = 0;
tend = 2*pi;
dt = 0.1;
tspan = tbeg:dt:tend; 
maxit = ceil((tend - tbeg) / dt);

%% Simdata config
fig2 = figure(2);
clf('reset');
global tau1p tau2p tau3p
subplot(3,1,1);
tau1p = animatedline('Color','k'); title('Ankle torque');
subplot(3,1,2);
tau2p = animatedline('Color','k'); title('Knee torque');
subplot(3,1,3); 
tau3p = animatedline('Color','k'); title('Lower back torque');

fig3 = figure(3);
clf('reset');
global tau1extp tau2extp tau3extp
subplot(3,1,1);
tau1extp = animatedline('Color','k'); title('External Ankle torque');
subplot(3,1,2);
tau2extp = animatedline('Color','k'); title('External Knee torque');
subplot(3,1,3); 
tau3extp = animatedline('Color','k'); title('External Lower back torque');

fig4 = figure(4);
clf('reset');
global hq1p hq2p hq3p eq1p eq2p eq3p
subplot(3,1,1); title('Ankle angle');
hq1p = animatedline('Color','r'); hold on;
eq1p = animatedline('Color','b'); legend('Human','Exo');
subplot(3,1,2); title('Knee angle')
hq2p = animatedline('Color','r'); hold on;
eq2p = animatedline('Color','b'); legend('Human','Exo');
subplot(3,1,3); title('Lower back angle')
hq3p = animatedline('Color','r'); hold on;
eq3p = animatedline('Color','b'); legend('Human','Exo');

fig5 = figure(5);
clf('reset');
global Hp HJp Tp gammaHp gammaHJp gammaTp
subplot(3,2,5);
Hp =  animatedline(); title('Hip spring magnitude');
subplot(3,2,6);
gammaHp = animatedline(); title('Hip spring angle');
subplot(3,2,3);
HJp = animatedline(); title('Hip Joint spring magnitude');
subplot(3,2,4);
gammaHJp = animatedline(); title('Hip Joint spring magnitude');
subplot(3,2,1);
Tp =  animatedline(); title('Torso spring magnitude');
subplot(3,2,2);
gammaTp =  animatedline(); title('Torso spring angle');

%% Solver config
fig1 = figure(1);
clf('reset');
opts = odeset('OutputFcn',@odeplot); 
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

% Human points of contact  
hHx =  L1*cos(hq1)+Lh*cos(hq2);
hHy  = L1*sin(hq1)+Lh*sin(hq2) ;

hHJx =  L1*cos(hq1)+L2*cos(hq2);
hHJy =  L1*sin(hq1)+L2*sin(hq2) ;   

hTx =  L1*cos(hq1)+L2*cos(hq2) + Lt*cos(hq3);
hTy =  L1*sin(hq1)+L2*sin(hq2) + Lt*sin(hq3);  

% corresponding exo points
eHx = L1*cos(eq1)+Lh*cos(eq2);
eHy =  L1*sin(eq1)+Lh*sin(eq2);

eHJx = L1*cos(eq1)+L2*cos(eq2);
eHJy = L1*sin(eq1)+L2*sin(eq2) ;

eTx = L1*cos(eq1)+L2*cos(eq2) + Lt*cos(eq3);
eTy =  L1*sin(eq1)+L2*sin(eq2) + Lt*sin(eq3);

%% Animation
fig6 = figure(6);
clf('reset');
human = animatedline('Marker','o','Color', 'black','LineWidth',1.25); hold on;    
exo =   animatedline('Marker','o','Color', 'blue','LineWidth',1.25);

tr = animatedline(); % end-effector trajectory
ecp = animatedline('Marker','o','MarkerSize',2,'Color','blue');
hcp = animatedline('Marker','o','MarkerSize',2,'Color','black');
ec1p = animatedline('Marker','o','MarkerSize',3,'Color','blue');
ec2p = animatedline('Marker','o','MarkerSize',3,'Color','blue');
ec3p = animatedline('Marker','o','MarkerSize',3,'Color','blue');

sprH = animatedline();
sprHJ = animatedline();
sprT = animatedline();

foot = animatedline('Marker','o');

box on;
axis equal;


last = numel(eO2x);
for i=1:last-1
    
    addpoints(exo, 0, 0);
    addpoints(exo, eO2x(i), eO2y(i));
    addpoints(exo, eO3x(i), eO3y(i));
    addpoints(exo, eO4x(i), eO4y(i));
    addpoints(tr,  eO4x(i), eO4y(i));
    
    addpoints(human, 0, 0);
    addpoints(human, hO2x(i), hO2y(i));
    addpoints(human, hO3x(i), hO3y(i));
    addpoints(human, hO4x(i), hO4y(i));
    
    addpoints(sprH, hHx(i), hHy(i)); 
    addpoints(sprH, eHx(i), eHy(i));
    addpoints(sprHJ, hHJx(i), hHJy(i)); 
    addpoints(sprHJ, eHJx(i), eHJy(i));
    addpoints(sprT, hTx(i), hTy(i)); 
    addpoints(sprT, eTx(i), eTy(i));   
    
    eC1 = [eO2x(i)/2; 
           eO2y(i)/2];

    eC2 = [(eO2x(i)+eO3x(i))/2; 
           (eO2y(i)+eO3y(i))/2 ];

    eC3 = [(eO3x(i)+eO4x(i))/2; 
           (eO3y(i)+eO4y(i))/2 ];
           
    eC = (m1*eC1 + m2*eC2 + m3*eC3) / (m1 + m2 + m3);
 
    hC1 = [hO2x(i)/2; 
           hO2y(i)/2];

    hC2 = [(hO2x(i)+hO3x(i))/2; 
           (hO2y(i)+hO3y(i))/2 ];

    hC3 = [(hO3x(i)+hO4x(i))/2; 
           (hO3y(i)+hO4y(i))/2 ];
           
    hC = (hm1*hC1 + hm2*hC2 + hm3*hC3) / (hm1 + hm2 + hm3);
    
    addpoints(ec1p, eC1(1), eC1(2));
    addpoints(ec2p, eC2(1), eC2(2));
    addpoints(ec3p, eC3(1), eC3(2));
    addpoints(ecp,  eC(1), eC(2));
    addpoints(hcp,  hC(1), hC(2));
    addpoints(foot, 0, 0);
    addpoints(foot, 0.24, 0);
    
    drawnow limitrate nocallbacks;

    clearpoints(foot);
    clearpoints(sprH);
    clearpoints(sprHJ);
    clearpoints(sprT);
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
addpoints(tr,  eO4x(last), eO4y(last));

addpoints(human, 0, 0);
addpoints(human, hO2x(last), hO2y(last));
addpoints(human, hO3x(last), hO3y(last));
addpoints(human, hO4x(last), hO4y(last));

addpoints(sprH, hHx(last), hHy(last)); 
addpoints(sprH, eHx(last), eHy(last));
addpoints(sprHJ, hHJx(last), hHJy(last)); 
addpoints(sprHJ, eHJx(last), eHJy(last));
addpoints(sprT, hTx(last), hTy(last)); 
addpoints(sprT, eTx(last), eTy(last));  

eC1 = [eO2x(last)/2; 
       eO2y(last)/2];

eC2 = [(eO2x(last)+eO3x(last))/2; 
       (eO2y(last)+eO3y(last))/2 ];

eC3 = [(eO3x(last)+eO4x(last))/2; 
       (eO3y(last)+eO4y(last))/2 ];

eC = (m1*eC1 + m2*eC2 + m3*eC3) / (m1 + m2 + m3);

addpoints(ec1p, eC1(1), eC1(2));
addpoints(ec2p, eC2(1), eC2(2));
addpoints(ec3p, eC3(1), eC3(2));
addpoints(ecp,  eC(1), eC(2));

addpoints(foot, 0, 0);
addpoints(foot, 0.24, 0);

drawnow;

%% ODE system in non-linear state-space representation
function dx = sys(t,x)

    global m1 m2 m3 L1 L2 L3 g Lh Lt 
    global Ch Chj Ct Bh Bhj Bt
    global tau1p tau2p tau3p
    global tau1extp tau2extp tau3extp
    global hq1p hq2p hq3p
    global eq1p eq2p eq3p
    global Hp HJp Tp
    global gammaHp gammaHJp gammaTp

    % State-space variables
    q = [x(1); x(2); x(3) ];
    qdot = [x(4); x(5); x(6)];
    
    if (q(2) - q(1) < 0)
        q(2) = q(1);
    end
    
    % Human trajectory in joint space
%     q_h = [ pi/2+pi/6*sin(t);
%             pi/2+pi/6*sin(t);
%             pi/2+pi/6*sin(t) ];
%         
%     qdot_h = [pi/6*cos(t);
%               pi/6*cos(t);
%               pi/6*cos(t) ];

    q_h = [pi/4 pi/2+pi/4 pi/4].';
    qdot_h = [0 0 0].';

    % Human points of contact  
    hH = [ L1*cos(q_h(1))+Lh*cos(q_h(2));
           L1*sin(q_h(1))+Lh*sin(q_h(2)) ];
    
    hHdot = [ -L1*sin(q_h(1))*qdot_h(1) - Lh*sin(q_h(2))*qdot_h(2);
               L1*cos(q_h(1))*qdot_h(1) + Lh*cos(q_h(2))*qdot_h(2) ];
         
    hHJ = [ L1*cos(q_h(1))+L2*cos(q_h(2));
            L1*sin(q_h(1))+L2*sin(q_h(2)) ];   
    
    hHJdot = [ -L1*sin(q_h(1))*qdot_h(1) - L2*sin(q_h(2))*qdot_h(2);
                L1*cos(q_h(1))*qdot_h(1) + L2*cos(q_h(2))*qdot_h(2) ];   
          
    hT = [ L1*cos(q_h(1))+L2*cos(q_h(2)) + Lt*cos(q_h(3));
           L1*sin(q_h(1))+L2*sin(q_h(2)) + Lt*sin(q_h(3))];  

    hTdot = [ -L1*sin(q_h(1))*qdot_h(1)-L2*sin(q_h(2))*qdot_h(2) - Lt*sin(q_h(3))*qdot_h(3);
               L1*cos(q_h(1))*qdot_h(1)+L2*cos(q_h(2))*qdot_h(2) + Lt*cos(q_h(3))*qdot_h(3)];  
         
    % corresponding exo points
    eH = [ L1*cos(q(1))+Lh*cos(q(2));
           L1*sin(q(1))+Lh*sin(q(2)) ];
    
    eHdot = [ -L1*sin(q(1))*qdot(1) - Lh*sin(q(2))*qdot(2);
               L1*cos(q(1))*qdot(1) + Lh*cos(q(2))*qdot(2) ];
         
    eHJ = [ L1*cos(q(1))+L2*cos(q(2));
            L1*sin(q(1))+L2*sin(q(2)) ];   
    
    eHJdot = [ -L1*sin(q(1))*qdot(1) - L2*sin(q(2))*qdot(2);
                L1*cos(q(1))*qdot(1) + L2*cos(q(2))*qdot(2) ];   
          
    eT = [ L1*cos(q(1))+L2*cos(q(2)) + Lt*cos(q(3));
           L1*sin(q(1))+L2*sin(q(2)) + Lt*sin(q(3))];  

    eTdot = [ -L1*sin(q(1))*qdot(1)-L2*sin(q(2))*qdot(2) - Lt*sin(q(3))*qdot(3);
               L1*cos(q(1))*qdot(1)+L2*cos(q(2))*qdot(2) + Lt*cos(q(3))*qdot(3)];  
       
    % Reaction forces
    H = Ch*(hH-eH) + Bh*(hHdot - eHdot);
    HJ = Chj*(hHJ - eHJ) + Bhj*(hHJdot - eHJdot);
    T  = Ct*(hT - eT) + Bt*(hTdot - eTdot);
    
    % Their magnitudes
    Tnorm = norm(T);
    Hnorm = norm(H);
    HJnorm = norm(HJ);
    Flnorm = 60;
    
    % Reaction forces absolute angles
    gammaT = acos(T(1)/Tnorm);
    gammaH = acos(H(1)/Hnorm);
    gammaHJ = acos(HJ(1)/HJnorm); 
    gammaFl = pi/2;

    % Check if any magnitude is zero
    gammaT(isnan(gammaT))=0;
    gammaH(isnan(gammaH))=0;
    gammaHJ(isnan(gammaHJ))=0;
    
    tau_d = [0 0 0].';
    

    tau_ext = [ -Flnorm*L1*sin(gammaFl-q(1)) + Tnorm*L1*sin(gammaT-q(1)) + HJnorm*L1*sin(gammaHJ-q(1)) + Hnorm*L1*sin(gammaH-q(1));
                -Flnorm*L2*sin(gammaFl-q(2)) + Tnorm*L2*sin(gammaT-q(2)) + HJnorm*L2*sin(gammaHJ-q(2)) +   Hnorm*Lh*sin(gammaH-q(2));
                -Flnorm*L3*sin(gammaFl-q(3)) + Tnorm*Lt*sin(gammaT-q(3)) ];

    tau = [0; 0; 0];
    
    % state-space representation
%     dx = [ x(4); x(5); x(6); -M(q) \ N(q,qdot)] + [zeros(3,1); M(q) \ (tau + tau_ext)]+ [zeros(3,1); -M(q) \ tau_d];
    dx = [ x(4); x(5); x(6); lsqminnorm(-M(q),N(q,qdot))] + [zeros(3,1); lsqminnorm(M(q),(tau + tau_ext))] + [zeros(3,1); lsqminnorm(-M(q),tau_d)];
    

    
    addpoints(hq1p, t, q_h(1));
    addpoints(hq2p, t, q_h(2));
    addpoints(hq3p, t, q_h(3));
 
    addpoints(eq1p, t, q(1));
    addpoints(eq2p, t, q(2));
    addpoints(eq3p, t, q(3));

    addpoints(Hp , t,  Hnorm);
    addpoints(HJp, t, HJnorm);
    addpoints(Tp , t,  Tnorm);
    
    addpoints(gammaTp,  t,  gammaT*(180/pi));
    addpoints(gammaHp,  t,  gammaH*(180/pi));
    addpoints(gammaHJp, t, gammaHJ*(180/pi));

    addpoints(tau1extp, t, tau_ext(1));
    addpoints(tau2extp, t, tau_ext(2));
    addpoints(tau3extp, t, tau_ext(3));

%     addpoints(tau1p, t, tau(1));
%     addpoints(tau2p, t, tau(2));
%     addpoints(tau3p, t, tau(3));
    
    drawnow limitrate nocallbacks;
    
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
    
% Friction
function ret = F(qdot)
   
    brkwy_trq = [50; 50; 50;];        
    brkwy_vel = [0.2; 0.2; 0.2];   
    Col_trq = [40; 40; 40];
    visc_coef = [0.002; 0.002; 0.002];
    
    static_scale = sqrt(2*exp(1)).*(brkwy_trq-Col_trq);
    static_thr = sqrt(2).*brkwy_vel;                     % Velocity threshold for static torque
    Col_thr = brkwy_vel./10;    
    
    ret = visc_coef .* qdot ...
         + static_scale .* (qdot./static_thr.*exp(-(qdot./static_thr).^2)) ...
         + Col_trq .* tanh(qdot./Col_thr); 
end

function ret = N(q,qdot)

    ret = V(q,qdot) + G(q) + F(qdot);

end
    
    
    