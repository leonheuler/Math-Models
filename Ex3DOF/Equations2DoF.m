clear all

syms L1 L2 L3 m1 m2 m3 g
syms t real
syms q1(t) 
syms q2(t) 

x1 = L1*cos(q1);
y1 = L1*sin(q1);
x2 = L1*cos(q1) + L2*cos(q1+q2);
y2 = L1*sin(q1) + L2*sin(q1+q2);

x1dot = diff(x1,t);
y1dot = diff(y1,t);
x2dot = diff(x2,t);
y2dot = diff(y2,t);

% Kinetic energy
K1 = m1/2*(x1dot^2+y1dot^2); 
K1 = expand(K1,'ArithmeticOnly',true);
K1 = combine(K1, 'sincos');

K2 = m2/2*(x2dot^2+y2dot^2);
K2 = expand(K2,'ArithmeticOnly',true);
K2 = combine(K2, 'sincos');
K = K1+K2;

% Potential energy
P1 = m1*g*y1;
P2 = m2*g*y2;
P = P1 + P2;

% Lagrangian
L = K - P;


% Partial derivatives of Kinetic energy 
syms dummy
dLq1dot = subs(diff(subs(L, diff(q1,t),dummy),dummy),dummy,diff(q1,t));
dLq2dot = subs(diff(subs(L, diff(q2,t),dummy),dummy),dummy,diff(q2,t));

% Partial derivatives of Lagrangian
dLq1 = subs(diff(subs(L, q1, dummy),dummy),dummy, q1);
dLq2 = subs(diff(subs(L, q2, dummy),dummy),dummy, q2);

% Lagrange Differential equation
diffEq1 = diff(dLq1dot,t) - dLq1;
diffEq2 = diff(dLq2dot,t) - dLq2;

% diffEq variables
variables = { q1, q2,  diff(q1,t), diff(q2,t),  diff(q1,t,2), diff(q2,t,2)  };

% desired variables 
syms q1 q2 q1dot q2dot q1ddot q2ddot
variablesshort = { q1, q2, q1dot, q2dot, q1ddot, q2ddot  };

% Lagrange differential equations with desired variables
DiffEq1 = subs(diffEq1, variables, variablesshort);
DiffEq2 = subs(diffEq2, variables, variablesshort);

eq1 = simplify(DiffEq1);
eq2 = simplify(DiffEq2);

% Gathering elements of matrix M from equations
tm11 = children(collect(eq1, q1ddot)); 
tm12 = children(collect(eq1, q2ddot)); 
tm21 = children(collect(eq2, q1ddot));
tm22 = children(collect(eq2, q2ddot));

M11 = collect(coeffs( tm11(1), q1ddot), [m1 m2]); 
M12 = collect(coeffs( tm12(1), q2ddot), [m1 m2]); 

M21 = collect(coeffs( tm21(1), q1ddot), [m1 m2]); 
M22 = collect(coeffs( tm22(1), q2ddot), [m1 m2]);

M = [ M11 M12;
      M21 M22 ];

% Gathering elements of matrix V from equations

tv11 = children(collect(eq1, q1dot)); 
tv12 = children(collect(eq1, q2dot));
tv21 = children(collect(eq2, q1dot)); 
tv22 = children(collect(eq2, q2dot));

tv11f = factor(tv11(1), q1dot);
tv12f = factor(tv12(1), q2dot);
tv21f = factor(tv21(1), q1dot);
tv22f = factor(tv22(1), q2dot);

V11 = prod(tv11f(length(tv11f)-1));
V12 = prod(tv12f(length(tv12f)-1));
V21 = prod(tv21f(length(tv21f)-1));
V22 = prod(tv22f(length(tv22f)-1));

 
% V11 = coeffs( expand(tv11(1)), q1dot);
% V12 = coeffs( expand(tv12(1)), q2dot);
% V21 = coeffs( expand(tv21(1)), q1dot);
% V22 = coeffs( expand(tv22(1)), q2dot);



V = [ V11 V12;
      V21 V22 ];
  
% Solve system of L diff. equations relative to angular accelerations
solution = solve(DiffEq1, DiffEq2,  q1ddot, q2ddot );

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;


% Q1DDOT = simplify(Q1DDOT);
% Q2DDOT = simplify(Q2DDOT);





