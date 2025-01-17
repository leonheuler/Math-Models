%% Frank L.Lewis p.248
% Convert the desired Cartesian trajectory yd(t) to a joint-space trajectory
% qd(t) using the inverse kinematics. Then use the joint-space computed-torque control schemes

%% Parameters
global m1 m2 L1 L2 g 
m1 = 5; m2 = 3; L1 = 0.5; L2 = 0.5; g = 9.81;

global Fv Fd
Fv = diag([4 4]);
Fd = diag([0.0 0.0]);

global Kp Kv Ki
Wn = 12;
Kp = diag([Wn^2 Wn^2]);
Kv = diag([2*Wn 2*Wn]);
Ki = diag([500 500]);


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

%% Friction force plot
    % fig7 = figure(7);
    % clf('reset');
    % title('Friction');
    % global f1p f2p
    % f1p = animatedline('Color','blue');
    % f2p = animatedline('Color', 'red');
    % grid on; axis auto;
    % legend('Friction1','Friction2');

% Initial conditions (Cartesian)
global r

r = 0.1;

x_d = r*sin(0);
y_d = -(L1+L2)+1.001*r-r*cos(0);

x_dot_d = r*cos(0);
y_dot_d = r*sin(0);  

% Inverse Kinematics
C =  (x_d^2 + y_d^2 - L1^2 - L2^2) / (2*L1*L2);
D = sqrt(1 - C^2);
q2_d = atan2(D,C);
q1_d =  atan2(y_d, x_d) - atan2(L2*sin(q2_d), L1+L2*cos(q2_d)); 

% Inidial Conditions (Joint space)
q_d = [ q1_d q2_d ]';
q_dot_d = invJ(q_d)*[x_dot_d y_dot_d]';

% State-space initial conditions vector
x0 = [q_d; q_dot_d; 0; 0];
%% Solver config  
fig1 = figure(1);
clf('reset');
opts_1 = odeset('Events', @events,'Stats','on','OutputFcn',@odeplot);
% opts_2 = odeset('RelTol',1e-2,'AbsTol',1e-4);
% opts = odeset(opts_1,opts_2);
opts = odeset(opts_1);

% Simulation time
tspan = [0 2*pi];


%% Solver  
tic; 
[t, x] = ode23s(@sys, tspan, x0, opts);
toc;


%% Robot


function dx = sys(t,x)
    
    global f1p f2p e1p e2p t1p t2p q1dp q2dp q1p q2p  trp trdp w1p w2p w1dp w2dp a1p a2p a1dp a2dp
    global Kp Kv Ki L1 L2 r

    % Feedback
    q = [ x(1) x(2) ]';
    q_dot = [ x(3) x(4) ]';
    e_integral = [x(5) x(6)]';
    
    % Desired trajectory in joint space
    x_d = r*sin(t);
    y_d = -(L1+L2)+1.001*r-r*cos(t);

    % Desired trajectory speed
    x_dot_d = r*cos(t);
    y_dot_d = r*sin(t);    

    % Desired trajectory acceleration
    x_dot_dot_d = -r*sin(t);
    y_dot_dot_d = r*cos(t);
    
    % Inverse Kinmatics
    C = (x_d^2 + y_d^2 - (L1^2 + L2^2)) / (2*L1*L2);
    D = sqrt(1 - C^2);

    q2_d = atan2(D,C);
    q1_d = atan2(y_d, x_d) - atan2(L2*sin(q2_d), L1+L2*cos(q2_d)); 
    
    q_d = [ q1_d q2_d ]';    
    q_dot_d = invJ(q)*[x_dot_d y_dot_d]';
    q_dot_dot_d = invJ(q)*[x_dot_dot_d y_dot_dot_d]' - invJ(q)*Jdot(q,q_dot)*q_dot_d;
    
    addpoints(q1p, t, q(1));
    addpoints(q2p, t, q(2));
    addpoints(q1dp, t, q_d(1));
    addpoints(q2dp, t, q_d(2));  
    
%     Ftrenie = F(q_dot);
%     addpoints(f1p, q(1), Ftrenie(1));
%     addpoints(f2p, q(2), Ftrenie(2));
    
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
    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    addpoints(e1p, t, e(1));
    addpoints(e2p, t, e(2));
    
    % PID joint-space computed-Torque without feedforward acceleration
%     tau =  M(q)*(q_dot_dot_d + Kp*e + Kv*e_dot + Ki*e_integral) + N(q, q_dot);
      
    % PD-plus-Gravity Control
%     tau =   Kp*e + Kv*e_dot + G(q);
    
    % Classical Joint Control (w=50)
    tau =  Kp*e + Kv*e_dot + Ki*e_integral;
    
    addpoints(t1p, t, tau(1));
    addpoints(t2p, t, tau(2));    
    
    % Non-linear state-space formulation
    dx = [ q_dot; -M(q)^-1*N(q, q_dot); e] + [zeros(2); M(q)^-1; zeros(2)]*tau;
    
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

function [value, isterminal, direction] = events(t,x)

value = [ x(3); x(4) ];
direction = [];
isterminal = [0; 0];
 
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
    
    w = q_dot;
    brkwy_trq = [5; 5];        
    brkwy_vel = [0.001; 0.001];   
    Col_trq = [4; 4];
    visc_coef = [0.001; 0.001];
    
    static_scale = sqrt(2*exp(1))*(brkwy_trq-Col_trq);
    static_thr = sqrt(2)*brkwy_vel;                     % Velocity threshold for static torque
    Col_thr = brkwy_vel./10;    
    
    ret = visc_coef .* w ...
         + static_scale .* (w./static_thr.*exp(-(w./static_thr).^2)) ...
         + Col_trq .* tanh(w./Col_thr); 
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










