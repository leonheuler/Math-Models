%% Computed torque control simulation
%   - state-space formulation;
%   - solver uses mass-matrix (no inversion)

%% Parameters
global m1 m2 L1 L2 g 
m1 = 1; m2 = 1; L1 = 1; L2 = 1; g = 9.81;

global Fv Fd
Fv = diag([0 0]);
Fd = diag([0 0]);

global Kp Kv Ki taud taumax
Wn = 50;
Kp = diag([Wn^2 Wn^2]);
Kv = diag([2*Wn 2*Wn]);
Ki = diag([0 0]);
taud = [1; 1];
taumax = 35;

%% Error plots
fig3 = figure(3);
title('Errors');
clf('reset');
global e1p e2p 
e1p = animatedline('Color', 'blue');
e2p = animatedline('Color', 'red');
grid on; legend('e_1','e_2');
%% Tau plots
fig4 = figure(4);
title('Tau');
clf('reset');
global t1p t2p
t1p = animatedline('Color', 'blue');
t2p = animatedline('Color', 'red');
grid on; legend('tau_1','tau_2');

%% Solver config  
fig1 = figure(1);
clf('reset');
opts_1 = odeset('Stats','on');
opts_2 = odeset('Mass',@MassMatrix,'MStateDependence','strong','OutputFcn',@odeplot);
opts_3 = odeset('RelTol',1e-3,'AbsTol',1e-5);
opts = odeset(opts_1,opts_2,opts_3);

tspan = [0 10];
x0 = [0.1 0 0 0]';
% x0 = [0.1 0 0 0 0 0]';


%% Solver  
tic; 
[t, x] = ode23t(@sys, tspan, x0, opts);
toc;


%% Fcn defs

function dx = sys(t,x)
    
    global e1p e2p t1p t2p
    global Kp Kv Ki taud taumax

    q = [ x(1) x(2) ]';
    q_dot = [ x(3) x(4) ]';
%     e_integral = [x(5) x(6)]';
    
    q_d = [ 0.1*sin(2*pi*t/2);
            0.1*cos(2*pi*t/2)];
        
    q_dot_d = [0.1*pi*cos(2*pi*t/2);
               -0.1*pi*sin(2*pi*t/2)];
    
    q_dot_dot_d = [-0.1*pi^2*sin(2*pi*t/2);
                   -0.1*pi^2*cos(2*pi*t/2)];


    e = q_d - q;
    e_dot = q_dot_d - q_dot;

    addpoints(e1p, t, e(1));
    addpoints(e2p, t, e(2));
    
    tau = M(q)*(q_dot_dot_d + Kp*e + Kv*e_dot) + N(q,q_dot);
%     tau = M(q)*(q_dot_dot_d + Kp*e + Kv*e_dot + Ki*e_integral) + N(q,q_dot);
 
%     if (abs(tau(1)) > taumax)
%         tau(1) = taumax*sign(tau(1));
%     end
%     
%     if (abs(tau(2)) > taumax)
%         tau(2) = taumax*sign(tau(2));
%     end
    
    addpoints(t1p, t, tau(1));
    addpoints(t2p, t, tau(2));    
    
    dx = cat(1, q_dot, -N(q, q_dot)) + cat(1,zeros(2,1), tau) + cat(1,zeros(2,1), -taud);
%     dx = [ q_dot; -(N(q, q_dot)); e] + [zeros(2); eye(2); zeros(2)]*tau + [zeros(2); -eye(2); zeros(2) ]*taud;
 
end


function ret = M(q)

    global m1 m2 L1 L2
    
    ret = [(m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q(2)),      m2*L2^2+m2*L1*L2*cos(q(2));
                   m2*L2^2+m2*L1*L2*cos(q(2)),                         m2*L2^2];
    
            
end

function ret = MassMatrix(t, q)
    global m1 m2 L1 L2 
    Mss = [(m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q(2)),      m2*L2^2+m2*L1*L2*cos(q(2));
                   m2*L2^2+m2*L1*L2*cos(q(2)),                         m2*L2^2]; 
    ret = blkdiag(eye(2), Mss);       
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














