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

% Model
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

q0 = [pi/2 pi/2 pi/2 0 0 0 ].'; % initial conditions
tspan = 0:0.01:2*pi; % simulation time

% plot state variables 
fig1 = figure(1);
clf('reset');
opts = odeset('OutputFcn',@odeplot); 

q = ode45(@sys, tspan, q0, opts);


X1 = L1*cos(q(:,1));
Y1 = L1*sin(q(:,1));
X2 = L1*cos(q(:,1)) + L2*cos(q(:,2));
Y2 = L1*sin(q(:,1)) + L2*sin(q(:,2));
X3 = L1*cos(q(:,1)) + L2*cos(q(:,2)) + L3*cos(q(:,3));
Y3 = L1*sin(q(:,1)) + L2*sin(q(:,2)) + L3*sin(q(:,3));


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

    global tau1p tau2p tau3p q1p q2p q3p q1dp q2dp q3dp
    
    
    q = [x(1); x(2); x(3) ];
    qdot = [x(4); x(5); x(6)];
    
    
    % Desired trajectory in joint space
    q_d = [ pi/2 - pi/6*(1-cos(t));   
           pi/2 + 2*pi/6*(1-cos(t));       
            pi/2 - pi/6*(1-cos(t)) ];
    
    qdot_d = [0; 0; 0];    
    
    tau_d = [0 0 0].';
    
    % Error
    e = q_d - q;
    edot = qdot_d - qdot;
    
    
    % PID config
    w = 100;
    Kp = diag([w^2 w^2 w^2]);
    Kv = diag([2*w 2*w 2*w]);
          
    tau = M(q)*(Kp*e + Kv*edot) + N(q,qdot);         % CTC
%     tau = Kp*e + Kv*e_dot + vecG;               % PD-plus-Gravity
%     tau = [0; 0; 0];
    
    dx = [ x(4); x(5); x(6); -M(q) \ N(q,qdot)] + [zeros(3,1); M(q) \ (tau + tau_ext)]+ [zeros(3,1); -M(q) \ tau_d];
    
                                 
    addpoints(tau1p, t,tau(1));
    addpoints(tau2p, t,tau(2));
    addpoints(tau3p, t,tau(3));
    
    addpoints(q1p, t, x(1));
    addpoints(q2p, t, x(2));
    addpoints(q3p, t, x(3));
    
    addpoints(q1dp, t, q_d(1));
    addpoints(q2dp, t, q_d(2));
    addpoints(q3dp, t, q_d(3));
    
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

    ret = V(q,qdot) + G(q);

end
    
    
    