clear all

tic
simulation = 0; % 0 - генерация функций M(q),N(q,qodt) в символьном виде для Solver.m

syms t
syms phi1(t) phi2(t) phi3(t)
syms q1 q2 q3 q1dot q2dot q3dot q1ddot q2ddot q3ddot

if (simulation == 1)
    L1 = 1; L2 = 1; L3 = 1;
    m1 = 1; m2 = 1; m3 = 1; g = 9.81;
else
    syms L1 L2 L3 m1 m2 m3 g
end


x1 = L1*sin(phi1);
y1 = L1*cos(phi1);
x2 = L1*sin(phi1) + L2*sin(-phi2+phi1);
y2 = L1*cos(phi1) + L2*cos(-phi2+phi1);
x3 = L1*sin(phi1) + L2*sin(-phi2+phi1) + L3*sin(phi3-phi2+phi1);
y3 = L1*cos(phi1) + L2*cos(-phi2+phi1) + L3*cos(phi3-phi2+phi1);

c1x =  x1/2; 
c1y =  y1/2; 
c2x =  (x1+x2)/2; 
c2y =  (y1+y2)/2;
c3x =  (x2+x3)/2; 
c3y =  (y2+y3)/2;

x1 = c1x; y1 = c1y;
x2 = c2x; y2 = c2y;
x3 = c3x; y3 = c3y;

fprintf('Calculating Jacobian...');
J = jacobian([subs(x3,[phi1,phi2,phi3],[q1,q2,q3]), subs(y3,[phi1,phi2,phi3],[q1,q2,q3])], [q1, q2, q3]);
fprintf('Completed\n');

x1dot = diff(x1,t);
y1dot = diff(y1,t);
x2dot = diff(x2,t);
y2dot = diff(y2,t);
x3dot = diff(x3,t);
y3dot = diff(y3,t);

% Kinetic energy
I1 = 1/3*m1*L1^2;
K1 = I1/2*q1dot^2;

I2 = 1/3*m2*L2^2;
K2 = I2/2*q2dot^2 + m2/2*(x2dot^2+y2dot^2);

I3 = 1/3*m3*L3^2;
K3 = I3/2*q3dot^2 + m3/2*(x3dot^2+y3dot^2);

K = K1+K2+K3;

% Potential energy
P1 = m1*g*y1;
P2 = m2*g*y2;
P3 = m3*g*y3;
P = P1 + P2 + P3;


% Lagrangian
fprintf('Calculating Lagrangian...');
L = K - P;
fprintf('Completed\n');
% L = simplify(L);

% Construction of diff eq-s of motion
oldvars = { phi1, phi2, phi3, diff(phi1,t), diff(phi2,t), diff(phi3,t), diff(phi1,t,t), diff(phi2,t,t), diff(phi3,t,t)  };
newvars = { q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot };
L = subs(L, oldvars, newvars); 

% dL / dqdot
dLq1dot = diff(subs(L, oldvars, newvars), q1dot); 
dLq2dot = diff(subs(L, oldvars, newvars), q2dot);
dLq3dot = diff(subs(L, oldvars, newvars), q3dot);

% d/dt(dL/dqdot)
dLq1dotdt = subs( diff(subs(dLq1dot, newvars, oldvars), t), oldvars, newvars);
dLq2dotdt = subs( diff(subs(dLq2dot, newvars, oldvars), t), oldvars, newvars);
dLq3dotdt = subs( diff(subs(dLq3dot, newvars, oldvars), t), oldvars, newvars);

% dL/dq
dLq1 = diff(L, q1);
dLq2 = diff(L, q2);
dLq3 = diff(L, q3);

% w = 10;

eq1 = dLq1dotdt - dLq1;% - w^2*(pi/12 - q1) - 2*w*(0-q1dot);
eq2 = dLq2dotdt - dLq2;% - w^2*(0 - q2) - 2*w*(0-q2dot);
eq3 = dLq3dotdt - dLq3;% - w^2*(0 - q3) - 2*w*(0-q3dot);

fprintf('Simplifying diff. equations...');
eq1 = simplify(eq1);
eq2 = simplify(eq2);
eq3 = simplify(eq3);
fprintf('Completed\n');

% Gathering elements of M from equations
fprintf('Collecting elements of matrix M...');
t11 = children(collect(eq1, q1ddot)); t12 = children(collect(eq1, q2ddot)); t13 = children(collect(eq1, q3ddot));
t21 = children(collect(eq2, q1ddot)); t22 = children(collect(eq2, q2ddot)); t23 = children(collect(eq2, q3ddot));
t31 = children(collect(eq3, q1ddot)); t32 = children(collect(eq3, q2ddot)); t33 = children(collect(eq3, q3ddot));

M11 = collect(coeffs( t11(1), q1ddot), [m1 m2 m3]); 
M21 = collect(coeffs( t21(1), q1ddot), [m1 m2 m3]); 
M31 = collect(coeffs( t31(1), q1ddot), [m1 m2 m3]); 

M12 = collect(coeffs( t12(1), q2ddot), [m1 m2 m3]); 
M22 = collect(coeffs( t22(1), q2ddot), [m1 m2 m3]);
M32 = collect(coeffs( t32(1), q2ddot), [m1 m2 m3]); 

M13 = collect(coeffs( t13(1), q3ddot), [m1 m2 m3]);
M23 = collect(coeffs( t23(1), q3ddot), [m1 m2 m3]);
M33 = collect(coeffs( t33(1), q3ddot), [m1 m2 m3]);

M = [ M11 M12 M12;
      M21 M22 M23;
      M31 M32 M33 ];
fprintf('Completed\n');

% Solve system of L diff. equations relative to angular accelerations
% solution = solve(DiffEq1, DiffEq2,  q1ddot, q2ddot );
fprintf('Solving and simplifying system of DE...');
solution = solve(eq1, eq2, eq3,  q1ddot, q2ddot , q3ddot);

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;
Q3DDOT = solution.q3ddot;

Q1DDOT = simplify(Q1DDOT);
Q2DDOT = simplify(Q2DDOT);
Q3DDOT = simplify(Q3DDOT);
fprintf('Completed\n');

% Calculate N vector
fprintf('Calculating vector N...');
N = -M*[Q1DDOT; Q2DDOT; Q3DDOT];
N = simplify(N);
fprintf('Completed\n');

% Calculate G vector
fprintf('Calculating vector G...');
G1 = diff(subs(P,[phi1 phi2 phi3], [q1 q2 q3]),q1);
G2 = diff(subs(P,[phi1 phi2 phi3], [q1 q2 q3]),q2);
G3 = diff(subs(P,[phi1 phi2 phi3], [q1 q2 q3]),q3);
G = [ G1; G2; G3 ];
fprintf('Completed\n');


if (simulation == 0)
    fprintf('Generating functions...');
    matlabFunction(N, 'File','N','Vars', [t g q1 q2 q3 q1dot q2dot q3dot m1 m2 m3 L1 L2 L3] );
    matlabFunction(M, 'File','M','Vars', [t g q1 q2 q3 m1 m2 m3 L1 L2 L3] );
    matlabFunction(J, 'File','J','Vars', [t q1 q2 q3 L1 L2 L3] );
    matlabFunction(G, 'File','G','Vars', [t g q1 q2 q3 m1 m2 m3 L1 L2 L3] );
    fprintf('Completed\n');
else 
    matlabFunction(N, 'File','N','Vars', [q1 q2 q3 q1dot q2dot q3dot] );
    matlabFunction(M, 'File','M','Vars', [q1 q2 q3] );
    
    str=['xdot=@(t,x)[x(4);x(5);x(6);',char(Q1DDOT),';',char(Q2DDOT),';',char(Q3DDOT),'];'];
    str = strrep(str, 'q1dot','x(4)');
    str = strrep(str, 'q2dot','x(5)');
    str = strrep(str, 'q3dot','x(6)');
    str = strrep(str, 'q1','x(1)');
    str = strrep(str, 'q2','x(2)');
    str = strrep(str, 'q3','x(3)');
    eval(str);

    fig1 = figure(1);
    clf('reset');
    opts = odeset('Stats','on','OutputFcn',@odeplot);
    
    % initial conditions ======================================================


    x0=[pi+pi/12 0 0 0 0 0]; 
    %% Solver  
    tic; 
    time=[0 10];
    [t,q]=ode23s(xdot,time,x0,opts);
%     tspan = 0:0.01:10;
%     q=ode4(xdot,tspan,x0);
    toc;

    X1 = L1*sin(q(:,1));
    Y1 = L1*cos(q(:,1));
    X2 = L1*sin(q(:,1)) + L2*sin(q(:,2)+q(:,1));
    Y2 = L1*cos(q(:,1)) + L2*cos(q(:,2)+q(:,1));
    X3 = L1*sin(q(:,1)) + L2*sin(q(:,2)+q(:,1)) + L3*sin(q(:,3)+q(:,2)+q(:,1));
    Y3 = L1*cos(q(:,1)) + L2*cos(q(:,2)+q(:,1)) + L3*cos(q(:,3)+q(:,2)+q(:,1));


    fig2 = figure(2);
    clf('reset');
    h = animatedline('Marker','o','LineWidth',1.5);
    tr = animatedline();
    axis equal;


    for i=1:numel(q(:,1))-1

        addpoints(h, 0, 0);
        addpoints(h, X1(i), Y1(i));
        addpoints(h, X2(i), Y2(i));
        addpoints(h, X3(i), Y3(i));
        addpoints(tr, X3(i), Y3(i));


        drawnow;
    %     pause(0.05);
        clearpoints(h);
    end

    last = numel(q(:,1));
    addpoints(h, 0, 0);
    addpoints(h, X1(last), Y1(last));
    addpoints(h, X2(last), Y2(last));
    addpoints(h, X3(last), Y3(last));

end
toc
% Friction
function ret = F(q_dot)
    
    w = q_dot;
    brkwy_trq = 100;        
    brkwy_vel = 0.5;   
    Col_trq = 60;
    visc_coef = 0.5;
    
    static_scale = sqrt(2*exp(1))*(brkwy_trq-Col_trq);
    static_thr = sqrt(2)*brkwy_vel;                     % Velocity threshold for static torque
    Col_thr = brkwy_vel/10;    
    
    ret = visc_coef * w ...
         + static_scale * (w/static_thr*exp(-(w/static_thr)^2)) ...
         + Col_trq * tanh(w/Col_thr); 
end





