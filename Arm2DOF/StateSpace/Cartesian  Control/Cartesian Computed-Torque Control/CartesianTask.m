%% Frank L.Lewis p.248
% Convert the desired Cartesian trajectory yd(t) to a joint-space trajectory
% qd(t) using the inverse kinematics. Then use the joint-space computed-torque control schemes

%% Parameters
global m1 m2 L1 L2 g 
m1 = 5; m2 = 3; L1 = 0.5; L2 = 0.5; g = 9.81;

global Fv Fd
Fv = diag([0 0]);
Fd = diag([00 0]); 

global Kp Kv Ki
Wn = 10;
Kp = diag([Wn^2 Wn^2]);
Kv = diag([2*Wn 2*Wn]);
Ki = diag([0 0]);


%% Error plots
fig3 = figure(3);
title('Errors');
clf('reset');
global e1p e2p 
e1p = animatedline('Color', 'blue','LineWidth',1.0);
e2p = animatedline('Color', 'red','LineWidth',1.0);
grid on; legend('e_1','e_2');
%% Tau plots
fig4 = figure(4);
title('Tau');
clf('reset');
global t1p t2p
t1p = animatedline('Color', 'blue','LineWidth',1.0);
t2p = animatedline('Color', 'red','LineWidth',1.0);
grid on; legend('tau_1','tau_2');
%% q plots
fig5 = figure(5);
clf('reset');
subplot(3,1,1);
title('q_d');
global q1dp q2dp q1p q2p
q1p = animatedline('Color', 'blue','LineWidth',1.0);
q2p = animatedline('Color', 'red','LineWidth',1.0);
q1dp = animatedline('Color', 'blue','LineStyle','--','LineWidth',1.0);
q2dp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('q1','q2','q1_d','q2_d');

subplot(3,1,2);
title('w_d');
% clf('reset');
global w1dp w2dp w1p w2p
w1p = animatedline('Color', 'blue','LineWidth',1.0);
w2p = animatedline('Color', 'red','LineWidth',1.0);
w1dp = animatedline('Color', 'blue','LineStyle','--','LineWidth',1.0);
w2dp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('w1','w2','w1_d','w2_d');

subplot(3,1,3);
title('a_d');
% clf('reset');
global a1dp a2dp a1p a2p
a1p = animatedline('Color', 'blue','LineWidth',1.0);
a2p = animatedline('Color', 'red','LineWidth',1.0);
a1dp = animatedline('Color', 'blue','LineStyle','--','LineWidth',1.0);
a2dp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('a1','a2','a1_d','a2_d');

%% Traj plots
fig6 = figure(6);
title('tr_d');
clf('reset');
global trp trdp
trp = animatedline('Color', 'blue','LineWidth',1.0);
trdp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('tr','tr_d');
axis equal;


%% Solver config  
fig1 = figure(1);
clf('reset');
opts_1 = odeset('Stats','on','OutputFcn',@odeplot);
opts_2 = odeset('RelTol',1e-3,'AbsTol',1e-5);
opts = odeset(opts_1,opts_2);

% Simulation time
tspan = [0 2*pi];

% Initial conditions (Cartesian)
global r

r = 0.1;

x_0 = r*sin(0);
y_0 = -(L1+L2)+1.001*r-r*cos(0);

x_dot_0 = r*cos(0);
y_dot_0 = r*sin(0);  

% Inverse Kinematics
% C =  (x_d^2 + y_d^2 - L1^2 - L2^2) / (2*L1*L2);
% D = sqrt(1 - C^2);
% q2_d = atan2(D,C);
% q1_d =  atan2(y_d, x_d) - atan2(L2*sin(q2_d), L1+L2*cos(q2_d)); 

% Inidial Conditions (Joint space)
q_0 = [ -pi/2 -0.001 ]';
q_dot_0 = invJ(q_0)*[x_dot_0 y_dot_0]';
% q_dot_d = [0 0]';


% State-space initial conditions vector
x0 = [q_0; q_dot_0; 0; 0];


%% Solver  
tic; 
[t, x] = ode23s(@sys, tspan, x0, opts);
toc;


%% Robot


function dx = sys(t,x)
    
    global e1p e2p t1p t2p q1dp q2dp q1p q2p  trp trdp w1p w2p w1dp w2dp a1p a2p a1dp a2dp 
    global Kp Kv Ki L1 L2 r

    % Feedback
    q = [ x(1) x(2) ]';
    q_dot = [ x(3) x(4) ]';
    ey_integral = [x(5) x(6)]';
    

    y = [ L1*cos(q(1)) + L2*cos(q(1)+q(2));
          L1*sin(q(1)) + L2*sin(q(1)+q(2)) ];

      
    ydot = [ -L1*sin(q(1))*q_dot(1) - L2*sin(q(1)+q(2))*(q_dot(1)+q_dot(2));
              L1*cos(q(1))*q_dot(1) + L2*cos(q(1) + q(2))*(q_dot(1) + q_dot(2)) ];
    
    % Desired trajectory in joint space
    x_d = r*sin(t);
    y_d = -(L1+L2)+1.001*r-r*cos(t);

    x_dot_d = r*cos(t);
    y_dot_d = r*sin(t);    

    x_dot_dot_d = -r*sin(t);
    y_dot_dot_d = r*cos(t);
    
    % Feedforward acceleration
    ydotdotd = [ x_dot_dot_d y_dot_dot_d ]';
    
    % Inverse Kinmatics
    C = (x_d^2 + y_d^2 - (L1^2 + L2^2)) / (2*L1*L2);
    D = -sqrt(1 - C^2);
    
    q2_d = atan2(D,C);
    q1_d = atan2(y_d, x_d) - atan2(L2*sin(q2_d), L1+L2*cos(q2_d)); 
    
    q_d = [ q1_d q2_d ]';    
    q_dot_d = invJ(q_d)*[x_dot_d y_dot_d]';
    q_dot_dot_d = invJ(q_d)*[x_dot_dot_d y_dot_dot_d]' - invJ(q)*Jdot(q_d,q_dot_d)*q_dot_d;
    
    addpoints(q1p, t, q(1));
    addpoints(q2p, t, q(2));
    addpoints(q1dp, t, q_d(1));
    addpoints(q2dp, t, q_d(2));  
    
    addpoints(w1p, t, q_dot(1));
    addpoints(w2p, t, q_dot(2));
    addpoints(w1dp, t, q_dot_d(1));
    addpoints(w2dp, t, q_dot_d(2));
    
    % Forward Kinematics (animation purposes only)
    r2 = [  L1*cos(q(1)) + L2*cos(q(1) + q(2));
            L1*sin(q(1)) + L2*sin(q(1) + q(2))];

    r2d = [ L1*cos(q_d(1)) + L2*cos(q_d(1) + q_d(2));
            L1*sin(q_d(1)) + L2*sin(q_d(1) + q_d(2))];
 
        
    addpoints(trp, r2(1), r2(2));
    addpoints(trdp, r2d(1), r2d(2));
    
    % Errors (joint space)
    ey = [ x_d y_d ]' - y;
    ey_dot = [ x_dot_d y_dot_d ]' - ydot;

    addpoints(e1p, t, ey(1));
    addpoints(e2p, t, ey(2));
    
    % PID cartesian-space computed-Torque 
%     tau =  M(q)*invJ(q)*(ydotdotd + Kp*ey + Kv*ey_dot + Ki*ey_integral) + N(q, q_dot);
      
    % PD-plus-Gravity Control
%     tau =  Kp*ey + Kv*ey_dot + G(q);

    % Classical Joint Control 
    tau =  Kp*ey + Kv*ey_dot + Ki*ey_integral;
    
    addpoints(t1p, t, tau(1));
    addpoints(t2p, t, tau(2));    
    
    % Non-linear state-space formulation
    dx = [ q_dot; -M(q)^-1*N(q, q_dot); ey] + [zeros(2); M(q)^-1; zeros(2)]*tau;
    
    q_dot_dot = [ dx(3); dx(4) ];
    addpoints(a1dp, t, q_dot_dot_d(1)); 
    addpoints(a2dp, t, q_dot_dot_d(2)); 
    addpoints(a1p, t, q_dot_dot(1)); 
    addpoints(a2p, t, q_dot_dot(2)); 
    
    % Linear state-space formulation
%     u = -M(q)^-1*(N(q,q_dot)) + M(q)^-1*tau; 
%     A = cat(2, zeros(4,2), cat(1, eye(2), zeros(2)));
%     B = cat(1, zeros(2), eye(2));
%     
%     dx = A*x + B*u;
                                  
end


function ret = M(q)
    global m1 m2 L1 L2 
    ret = [(m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q(2)),      m2*L2^2+m2*L1*L2*cos(q(2));
                   m2*L2^2+m2*L1*L2*cos(q(2)),                         m2*L2^2];        
end


function ret = N(q, q_dot)    
    ret = V(q,q_dot) + F(q_dot) + G(q);
end

function ret = V(q, q_dot)    
    global m1 m2 L1 L2 
    ret = [ -m2*L1*L2*(2*q_dot(1)*q_dot(2)+(q_dot(2)^2))*sin(q(2));
            m2*L1*L2*(q_dot(1)^2)*sin(q(2))];   
end

function ret = F(q_dot)
    global Fv Fd
    ret = Fv*q_dot + Fd*sign(q_dot); 
end

function ret = G(q)   
    global m1 m2 L1 L2 g   
    ret = [(m1+m2)*g*L1*cos(q(1)) + m2*g*L2*cos(q(1)+q(2));
            m2*g*L2*cos(q(1)+q(2))];
end


function ret = invJ(q)
    
    global L1 L2 
    ret = [                       cos(q(1) + q(2))/(L1*sin(q(2))),                       sin(q(1) + q(2))/(L1*sin(q(2)));
           -(L2*cos(q(1) + q(2)) + L1*cos(q(1)))/(L1*L2*sin(q(2))), -(L2*sin(q(1) + q(2)) + L1*sin(q(1)))/(L1*L2*sin(q(2))) ];

end

function ret = Jdot(q, q_dot)
    
    global L1 L2 
    ret = [   -L2*cos(q(1)+q(2))*(q_dot(1) + q_dot(2))-L1*cos(q(1))*q_dot(1),            -L2*cos(q(1)+q(2))*(q_dot(1)+q_dot(2));
              -L2*sin(q(1)+q(2))*(q_dot(1)+q_dot(2)) - L1*sin(q(1))*sin(q_dot(1)),       -L2*sin(q(1)+q(2))*(q_dot(1)+q_dot(2)) ];

end










