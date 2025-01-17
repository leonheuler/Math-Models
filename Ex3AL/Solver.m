clear all
%% Parameters
global m1 m2 m3 L1 L2 L3 g

m = 70;
H = 1.7;
L2 = 0.2450*H;
L1 = 0.2460*H;
L3 = H - (L1 + L2);

m1 = 0.0475*m;
m2 = 0.105*m;
m3 = 0.551*m/2;

g = 9.81;

% maxon 496660
global Km R Kb Im Ra L B
Km = diag(ones(1,3)*37.5e-3);
Kb = diag(ones(1,3)*255*(2*pi/60));
Im = diag(ones(1,3)*(44+5)*1e-3*1e-2^2);
R = diag(ones(1,3)*(1/113));
Ra = diag(ones(1,3)*0.236);
L = diag(ones(1,3)*0.169e-3);
B = diag(ones(1,3)*0.0002);

%% Plots
fig3 = figure(3);
clf('reset');
global tau1p tau2p tau3p
tau1p = animatedline('Color','red');
tau2p = animatedline('Color','blue');
tau3p = animatedline('Color','black');
legend('tau1','tau2','tau3'); grid on;

fig4 = figure(4);
clf('reset');
global q1p q2p q3p q1dp q2dp q3dp
subplot(3,1,1);
q1p = animatedline('Color','blue');
q1dp = animatedline('Color','red');
grid on; legend('q1','q1_d');
subplot(3,1,2);
q2p = animatedline('Color','blue');
q2dp = animatedline('Color','red');
grid on; legend('q2','q2_d');
subplot(3,1,3);
q3p = animatedline('Color','blue');
q3dp = animatedline('Color','red');
grid on; legend('q3','q3_d');

fig5 = figure(5);
clf('reset'); 
global i1p i2p i3p
subplot(3,1,1);
i1p = animatedline(); grid on; title('I1');
subplot(3,1,2);
i2p = animatedline(); grid on; title('I2');
subplot(3,1,3);
i3p = animatedline(); grid on; title('I3');

fig6 = figure(6);
clf('reset'); title('Voltage');
global u1p u2p u3p
subplot(3,1,1);
u1p = animatedline(); grid on; title('U1');
subplot(3,1,2);
u2p = animatedline(); grid on; title('U2');
subplot(3,1,3);
u3p = animatedline(); grid on; title('U3');

fig7 = figure(7);
clf('reset'); 
global m1p m2p m3p
subplot(3,1,1);
m1p = animatedline(); grid on; title('thetadot1');
subplot(3,1,2);
m2p = animatedline(); grid on; title('thetadot2');
subplot(3,1,3);
m3p = animatedline(); grid on; title('thetadot3');


%% Solver config  
fig1 = figure(1);
clf('reset');
opts_1 = odeset('Stats','on','OutputFcn',@odeplot);
opts_2 = odeset('RelTol',1e-3,'AbsTol',1e-5);
opts_3 = odeset('NonNegative',[1,2]);
% opts_4 = odeset('Mass',@MassMatrixFcn,'MStateDependence','strong');

% opts = odeset(opts_2,opts_3);
opts = odeset(opts_1,opts_2,opts_3); 

q0 = zeros(15,1);
% q0 = [0 0 0 0 0 0 0 ]';   
tic; 
tspan = 0:0.01:pi; 
[t, q] = ode23s(@sys, tspan, q0,opts);
% [t, q] = ode45(@sys, tspan, q0,opts);
% q = ode4(@sys, tspan, q0);
toc;
%%

X1 = L1*sin(q(:,1));
Y1 = L1*cos(q(:,1));
X2 = L1*sin(q(:,1)) + L2*sin(-q(:,2)+q(:,1));
Y2 = L1*cos(q(:,1)) + L2*cos(-q(:,2)+q(:,1));
X3 = L1*sin(q(:,1)) + L2*sin(-q(:,2)+q(:,1)) + L3*sin(q(:,3)-q(:,2)+q(:,1));
Y3 = L1*cos(q(:,1)) + L2*cos(-q(:,2)+q(:,1)) + L3*cos(q(:,3)-q(:,2)+q(:,1));


%%
% Animation
fig2 = figure(2);
clf('reset');
h = animatedline('Marker','o','LineWidth',1.5);
tr = animatedline();
cp = animatedline('Marker','o','MarkerSize',1,'Color','red');
c1p = animatedline('Marker','o','Color','blue');
c2p = animatedline('Marker','o','Color','blue');
c3p = animatedline('Marker','o','Color','blue');
axis equal;



last = numel(q(:,1));
for i=1:last-1

    addpoints(h, 0, 0);
    addpoints(h, X1(i), Y1(i));
    addpoints(h, X2(i), Y2(i));
    addpoints(h, X3(i), Y3(i));
    addpoints(tr, X3(i), Y3(i));
    
    rC1 = [X1(i)/2;Y1(i)/2];
    rC2 = [(X1(i)+X2(i))/2; (Y1(i)+Y2(i))/2];
    rC3 = [(X2(i)+X3(i))/2; (Y2(i)+Y3(i))/2];
    rC = (m1*[X1(i)/2;Y1(i)/2] + m2*[(X2(i)+X1(i))/2;(Y2(i)+Y1(i))/2] + m3*[(X2(i)+X3(i))/2;(Y3(i)+Y2(i))/2]) / (m1 + m2 + m3);
    
    addpoints(c1p, rC1(1), rC1(2));
    addpoints(c2p, rC2(1), rC2(2));
    addpoints(c3p, rC3(1), rC3(2));
    addpoints(cp, rC(1), rC(2));
    
    drawnow limitrate;
    
    clearpoints(c1p);
    clearpoints(c2p);
    clearpoints(c3p);
    clearpoints(h);
end
addpoints(h, 0, 0);
addpoints(h, X1(last), Y1(last));
addpoints(h, X2(last), Y2(last));
addpoints(h, X3(last), Y3(last));

%% ODE system in non-linear state-space representation
function dx = sys(t,x)
    
    % Non-linear state-space formulation
    % Generic
    global m1 m2 m3 L1 L2 L3 g
    global tau1p tau2p tau3p 
    global q1p q2p q3p q1dp q2dp q3dp 
    global i1p i2p i3p u1p u2p u3p 
    global m1p m2p m3p
    global Km R Kb Im Ra L B
    
    q = [x(1); x(2); x(3) ];
    qdot = [x(4); x(5); x(6)];
    i = [x(7); x(8); x(9)];
    qmdot = [x(10); x(11); x(12)];
    e_i_integral = [x(13); x(14); x(15)];
    
    matM = M(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);
    vecV = V(t,x(1),x(2),x(3),x(4),x(5),x(6),m1,m2,m3,L1,L2,L3);
    vecF = F(qdot);
    vecG = G(t, g, x(1),x(2),x(3),m1,m2,m3,L1,L2,L3);
    vecN = vecV + vecG + vecF;
    
%     tau_d = transpose(J(t,x(1),x(2),x(3),L1,L2,L3))*[0; 0];

    % Desired trajectory in joint space
    q_d = [ pi/6*(1-cos(t));   
           2*pi/6*(1-cos(t));       
            pi/6*(1-cos(t)) ];
    
    qdot_d = [pi/6*sin(t); 2*pi/6*sin(t); pi/6*sin(t)];    
        
    % Error
    e = q_d - q;
    edot = qdot_d - qdot;
    
    % PID config
    w = 100;
    Kp = diag([w^2 w^2 w^2]);
    Kv = diag([2*w 2*w 2*w]);
          
    tau = matM*(Kp*e + Kv*edot) + vecN;         % CTC
%     tau = Kp*e + Kv*e_dot + vecG;               % PD-plus-Gravity
%     tau = [0; 0; 0];

    % Saturation limit
%     TAU_MAX = 500;
%     tau(1) = constrain(tau(1),-TAU_MAX,TAU_MAX);
%     tau(2) = constrain(tau(2),-TAU_MAX,TAU_MAX);
%     tau(3) = constrain(tau(3),-TAU_MAX,TAU_MAX);
    
    i_d = Km \ R*tau;
    e_i = i_d - i; 
    
    Kp_i = diag(ones(1,3)*2000);
    Ki_i = diag(ones(1,3)*200);
    u = Kp_i*e_i + Ki_i*e_i_integral;
    
    u(1) = constrain(u(1), -24, 24);
    u(2) = constrain(u(2), -24, 24);
    u(3) = constrain(u(3), -24, 24);
    
    dx = [ x(4); x(5); x(6); -matM \ vecN; -L \ (Kb*qmdot+Ra*i); -Im \ (B*qmdot + R*tau); e_i ] + ...
         [zeros(3,1); matM \ R \ Km*i; L \ u; Im \ Km*i; zeros(3,1)];
    
                                 
    addpoints(tau1p, t,tau(1));
    addpoints(tau2p, t,tau(2));
    addpoints(tau3p, t,tau(3));
    
    addpoints(i1p, t,i(1));
    addpoints(i2p, t, i(2));
    addpoints(i3p,  t, i(3));
 
    addpoints(u1p, t, u(1));
    addpoints(u2p, t, u(2));
    addpoints(u3p,  t, u(3));
    
    addpoints(m1p, t, qmdot(1) * (60/2*pi));
    addpoints(m2p, t, qmdot(2) * (60/2*pi));
    addpoints(m3p,  t, qmdot(3) * (60/2*pi));
    
    addpoints(q1p, t, x(1));
    addpoints(q2p, t, x(2));
    addpoints(q3p, t, x(3));
    addpoints(q1dp, t, q_d(1));
    addpoints(q2dp, t, q_d(2));
    addpoints(q3dp, t, q_d(3));

end



function ret = constrain(val, min, max)
    
    if (val < min)
        ret = min;
    elseif (val > max)
        ret = max;
    else
        ret = val;
    end

end












