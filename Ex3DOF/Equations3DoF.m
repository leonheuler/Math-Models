syms L1 L2 L3 m1 m2 m3 g
syms t
syms q1(t) q2(t) q3(t)

x1 = L1*sin(q1);
y1 = L1*cos(q1);
x2 = L1*sin(q1) - L2*sin(q2-q1);
y2 = L1*cos(q1) + L2*cos(q2-q1);
x3 = L1*sin(q1) - L2*sin(q2-q1) - L3*sin(-q3+q2-q1);
y3 = L1*cos(q1) + L2*cos(q2-q1) + L3*cos(-q3+q2-q1);

x1dot = diff(x1,t);
y1dot = diff(y1,t);
x2dot = diff(x2,t);
y2dot = diff(y2,t);
x3dot = diff(x3,t);
y3dot = diff(y3,t);

% Kinetic energy
K1 = m1/2*(x1dot^2+y1dot^2);
K2 = m2/2*(x2dot^2+y2dot^2);
K3 = m3/2*(x3dot^2+y3dot^2);
K = K1 + K2 + K3;

% Potential energy
P1 = m1*g*y1;
P2 = m2*g*y2;
P3 = m3*g*y3;
P = P1 + P2 + P3;

% Lagrangian
L = K - P;

% Partial derivatives of Kinetic energy 
syms dummy
dLq1dot = subs(diff(subs(L, diff(q1,t),dummy),dummy),dummy,diff(q1,t));
dLq2dot = subs(diff(subs(L, diff(q2,t),dummy),dummy),dummy,diff(q2,t));
dLq3dot = subs(diff(subs(L, diff(q3,t),dummy),dummy),dummy,diff(q3,t));

% Partial derivatives of Lagrangian
dLq1 = subs(diff(subs(L, q1, dummy),dummy),dummy, q1);
dLq2 = subs(diff(subs(L, q2, dummy),dummy),dummy, q2);
dLq3 = subs(diff(subs(L, q3, dummy),dummy),dummy, q3);


% Lagrange Differential equation
diffEq1 = diff(dLq1dot,t) - dLq1;
diffEq2 = diff(dLq2dot,t) - dLq2;
diffEq3 = diff(dLq3dot,t) - dLq3;


% diffEq variables
variables = { q1, q2, q3, diff(q1,t), diff(q2,t), diff(q3,t), diff(q1,t,2), diff(q2,t,2), diff(q3,t,2) };

% desired variables 
syms q1 q2 q3 q1dot q2dot q3dot q1ddot q2ddot q3ddot
variablesshort = { q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot };

% Lagrange differential equations with desired variables
DiffEq1 = subs(diffEq1, variables, variablesshort);
DiffEq2 = subs(diffEq2, variables, variablesshort);
DiffEq3 = subs(diffEq3, variables, variablesshort);

eq1 = simplify(DiffEq1);
eq2 = simplify(DiffEq2);
eq3 = simplify(DiffEq3);

% Gathering elements of M from equations
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

% Solve system of L diff. equations relative to angular accelerations
solution = solve(DiffEq1, DiffEq2, DiffEq3, q1ddot, q2ddot, q3ddot);

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;
Q3DDOT = solution.q3ddot;
% 
% Q1DDOT = simplify(Q1DDOT);
% Q2DDOT = simplify(Q2DDOT);
% Q3DDOT = simplify(Q3DDOT);




