%% Frank L.Lewis p.248
% Convert the desired Cartesian trajectory yd(t) to a joint-space trajectory
% qd(t) using the inverse kinematics. Then use the joint-space computedtorque control schemes

%% Parameters
global m1 m2 L1 L2 g 
m1 = 5; m2 = 3; L1 = 0.5; L2 = 0.5; g = 9.81;

global Fv Fd
Fv = diag([5 5]);
Fd = diag([1 1]);

global Kp Kv Ki
Wn = 100;
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
title('q_d');
clf('reset');
global q1dp q2dp q1p q2p
q1p = animatedline('Color', 'blue','LineWidth',1.0);
q2p = animatedline('Color', 'red','LineWidth',1.0);
q1dp = animatedline('Color', 'blue','LineStyle','--','LineWidth',1.0);
q2dp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('q1','q2','q1_d','q2_d');
%% Traj plots
fig6 = figure(6);
title('tr_d');
clf('reset');
global trp trdp
trp = animatedline('Color', 'blue','LineWidth',1.0);
trdp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('tr','tr_d');
axis equal;
%% w plots
fig7 = figure(7);
title('q_d');
clf('reset');
global w1dp w2dp w1p w2p
w1p = animatedline('Color', 'blue','LineWidth',1.0);
w2p = animatedline('Color', 'red','LineWidth',1.0);
w1dp = animatedline('Color', 'blue','LineStyle','--','LineWidth',1.0);
w2dp = animatedline('Color', 'red','LineStyle','--','LineWidth',1.0);
grid on; legend('w1','w2','w1_d','w2_d');

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
x_d = r*sin(0);
y_d = -(L1+L2)+1.2*r-r*cos(0);
x_dot_d = r*cos(0);
y_dot_d = r*sin(0); 

% Inverse Kinematics
C =  (x_d^2 + y_d^2 - L1^2 - L2^2) / (2*L1*L2);
D = sqrt(1 - C^2);
q2 = atan2(D,C);
q1 =  atan2(y_d, x_d) - atan2(L2*sin(q2), L1+L2*cos(q2)); 

% Inidial Conditions (Joint space)
q = [ q1 q2 ]';
q_dot = invJ(q)*[x_dot_d y_dot_d]';

% Forward kinematics
y = [ L1*cos(q(1)) + L2*cos(q(1)+q(2));
      L1*sin(q(1)) + L2*sin(q(1)+q(2)) ];


ydot = [ -L1*sin(q(1))*q_dot(1) - L2*sin(q(1)+q(2))*(q_dot(1)+q_dot(2));
          L1*cos(q(1))*q_dot(1) + L2*cos(q(1) + q(2))*(q_dot(1) + q_dot(2)) ];

% State-space initial conditions vector
x0 = [y; ydot; 0; 0; q];


%% Solver  
tic; 
[t, x] = ode45(@sys, tspan, x0, opts);
toc;


%% Robot


function dx = sys(t,x)
    
    global e1p e2p t1p t2p q1dp q2dp q1p q2p  trp trdp w1p w2p w1dp w2dp
    global Kp Kv Ki L1 L2
    global r
    % Feedback
    y = [ x(1) x(2) ]';
    ydot = [ x(3) x(4) ]';
    e_integral = [x(5) x(6)]';
    q = [ x(7) x(8) ];
      
    q_dot = invJ(q)*ydot;
    
    
   % Desired trajectory in cartesian space
    x_d = r*sin(t);
    y_d = -(L1+L2)+1.2*r-r*cos(t);

    x_dot_d = r*cos(t);
    y_dot_d = r*sin(t);    

    x_dot_dot_d = -r*sin(t);
    y_dot_dot_d = r*cos(t);
    ydotdotd = [ x_dot_dot_d y_dot_dot_d ]';
    
    
    % Inverse Kinmatics (��� �������� ����������� (x_d(t),y_d(t)) -> (q1_d(t), q2_d(t)))
    C = (x_d^2 + y_d^2 - (L1^2 + L2^2)) / (2*L1*L2);
    D = sqrt(1 - C^2);

    q2_d = atan2(D,C);
    q1_d = atan2(y_d, x_d) - atan2(L2*sin(q2_d), L1+L2*cos(q2_d)); 
    
    q_d = [ q1_d q2_d ]';    
    q_dot_d = invJ(q)*[x_dot_d y_dot_d]';
    
%     q_dot_dot_d = invJ(q)*[x_dot_dot_d y_dot_dot_d]' - invJ(q)*Jdot(q,q_dot)*q_dot_d;
   
    addpoints(q1p, t, q(1));
    addpoints(q2p, t, q(2));
    addpoints(q1dp, t, q_d(1));
    addpoints(q2dp, t, q_d(2));  
    
    addpoints(w1p, t, q_dot(1));
    addpoints(w2p, t, q_dot(2));
    addpoints(w1dp, t, q_dot_d(1));
    addpoints(w2dp, t, q_dot_d(2));
    
    % Forward (animation purposes only)
    r2 = [  L1*cos(q(1)) + L2*cos(q(1) + q(2));
            L1*sin(q(1)) + L2*sin(q(1) + q(2))];

    r2d = [ L1*cos(q_d(1)) + L2*cos(q_d(1) + q_d(2));
            L1*sin(q_d(1)) + L2*sin(q_d(1) + q_d(2))];
 
        
    addpoints(trp, r2(1), r2(2));
    addpoints(trdp, r2d(1), r2d(2));
    
    % Errors
    e = [ x_d y_d ]' - y;
    e_dot = [ x_dot_d y_dot_d ]' - ydot;
    
    addpoints(e1p, t, e(1));
    addpoints(e2p, t, e(2));
    
    % PID joint-space computed-Torque without feedforward acceleration
%     f =  transpose(invJ(q)) * ( M(q)*(invJ(q_d)*ydotdotd - invJ(q_d)*Jdot(q_d,q_dot_d)*q_dot_d + Kp*e + Kv*e_dot + Ki*e_integral) + N(q, q_dot) );
      
    % PD-plus-Gravity Control
%     f = Kp*e + Kv*e_dot + transpose(invJ(q))*G(q);
    
    % Classical Joint Control
    f = Kp*e + Kv*e_dot + Ki*e_integral; 
    
    tau = transpose(Jac(q))*f;
    
    addpoints(t1p, t, tau(1));
    addpoints(t2p, t, tau(2));    
    
    % Non-linear state-space formulation
    dx = [ ydot; -My(q)^-1*Ny(q, q_dot); e; q_dot] + [zeros(2); My(q)^-1; zeros(2); zeros(2)]*f;

    % Linear state-space formulation
%     u = -M(q)^-1*(N(q,q_dot)) + M(q)^-1*tau; 
%     A = cat(2, zeros(4,2), cat(1, eye(2), zeros(2)));
%     B = cat(1, zeros(2), eye(2));
%     
%     dx = A*x + B*u;
                                  
end

function ret = My(q)

    ret = transpose(invJ(q))*M(q)*invJ(q);

end

function ret = Ny(q, q_dot)
    
    ret = transpose(invJ(q))*(  N(q,q_dot) - M(q)*invJ(q)*Jdot(q, q_dot)*q_dot  );

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

function ret = Jac(q)

    global L1 L2
    
    ret = [ - L2*sin(q(1) + q(2)) - L1*sin(q(1)), -L2*sin(q(1) + q(2));
             L2*cos(q(1) + q(2)) + L1*cos(q(1)),  L2*cos(q(1) + q(2)) ];

end








