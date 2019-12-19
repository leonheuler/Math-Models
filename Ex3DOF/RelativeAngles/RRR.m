clear all

tic

syms t
syms phi1(t) phi2(t) phi3(t)
syms q1 q2 q3 q1dot q2dot q3dot q1ddot q2ddot q3ddot
syms L1 L2 L3 m1 m2 m3 g

x1 = L1*sin(phi1);
y1 = L1*cos(phi1);
x2 = L1*sin(phi1) + L2*sin(-phi2+phi1);
y2 = L1*cos(phi1) + L2*cos(-phi2+phi1);
x3 = L1*sin(phi1) + L2*sin(-phi2+phi1) + L3*sin(phi3-phi2+phi1);
y3 = L1*cos(phi1) + L2*cos(-phi2+phi1) + L3*cos(phi3-phi2+phi1);

c1x = x1/2; 
c1y = y1/2; 
c2x = (x1+x2)/2; 
c2y = (y1+y2)/2;
c3x = (x2+x3)/2; 
c3y = (y2+y3)/2;

x1 = c1x; y1 = c1y;
x2 = c2x; y2 = c2y;
x3 = c3x; y3 = c3y;

fprintf('Calculating Jacobian...');
J = jacobian([subs(x3,[phi1,phi2,phi3],[q1,q2,q3]), subs(y3,[phi1,phi2,phi3],[q1,q2,q3])], [q1, q2, q3]);
fprintf('Completed!\n');

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
fprintf('Completed!\n');

% Construction of diff eq-s of motion
oldvars = { phi1, phi2, phi3, diff(phi1,t), diff(phi2,t), diff(phi3,t), diff(phi1,t,t), diff(phi2,t,t), diff(phi3,t,t)  };
newvars = { q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot };
K = subs(K, oldvars, newvars); 

% dL / dqdot
dKq1dot = diff(subs(K, oldvars, newvars), q1dot); 
dKq2dot = diff(subs(K, oldvars, newvars), q2dot);
dKq3dot = diff(subs(K, oldvars, newvars), q3dot);

% d/dt(dL/dqdot)
dKq1dotdt = subs( diff(subs(dKq1dot, newvars, oldvars), t), oldvars, newvars);
dKq2dotdt = subs( diff(subs(dKq2dot, newvars, oldvars), t), oldvars, newvars);
dKq3dotdt = subs( diff(subs(dKq3dot, newvars, oldvars), t), oldvars, newvars);

% dL/dq
dKq1 = diff(K, q1);
dKq2 = diff(K, q2);
dKq3 = diff(K, q3);


eq1 = dKq1dotdt - dKq1;
eq2 = dKq2dotdt - dKq2;
eq3 = dKq3dotdt - dKq3;

fprintf('Simplifying diff. equations...');
eq1 = simplify(eq1);
eq2 = simplify(eq2);
eq3 = simplify(eq3);
fprintf('Completed!\n');

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
fprintf('Completed!\n');

% Solve system of L diff. equations relative to angular accelerations
fprintf('Solving and simplifying system of DE...');
solution = solve(eq1, eq2, eq3,  q1ddot, q2ddot , q3ddot);

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;
Q3DDOT = solution.q3ddot;

Q1DDOT = simplify(Q1DDOT);
Q2DDOT = simplify(Q2DDOT);
Q3DDOT = simplify(Q3DDOT);
fprintf('Completed!\n');

% Calculate V vector
fprintf('Calculating vector V...');
V = -M*[Q1DDOT; Q2DDOT; Q3DDOT];
V = simplify(V);
fprintf('Completed!\n');

% Calculate G vector
fprintf('Calculating vector G...');
G1 = diff(subs(P,[phi1 phi2 phi3], [q1 q2 q3]),q1);
G2 = diff(subs(P,[phi1 phi2 phi3], [q1 q2 q3]),q2);
G3 = diff(subs(P,[phi1 phi2 phi3], [q1 q2 q3]),q3);
G = [ G1; G2; G3 ];
fprintf('Completed!\n');


fprintf('Generating functions...');
matlabFunction(V, 'File','V','Vars', [t q1 q2 q3 q1dot q2dot q3dot m1 m2 m3 L1 L2 L3] );
matlabFunction(M, 'File','M','Vars', [t g q1 q2 q3 m1 m2 m3 L1 L2 L3] );
matlabFunction(J, 'File','J','Vars', [t q1 q2 q3 L1 L2 L3] );
matlabFunction(G, 'File','G','Vars', [t g q1 q2 q3 m1 m2 m3 L1 L2 L3] );
fprintf('Completed!\n');

toc





