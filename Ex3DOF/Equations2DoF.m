clear all

syms L1 L2 L3 m1 m2 m3 g
% L1 = 1; L2 = 1; m1 = 1; m2 = 1; g = 9.81;

syms t real
syms phi1(t) 
syms phi2(t) 

x1 = L1*sin(phi1);
y1 = L1*cos(phi1);
x2 = L1*sin(phi1) + L2*sin(phi1+phi2);
y2 = L1*cos(phi1) + L2*cos(phi1+phi2);

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
% Construction of diff eq-s of motion
oldvars = { phi1, phi2, phi3, diff(phi1,t), diff(phi2,t), diff(phi3,t), diff(phi1,t,2), diff(phi2,t,2), diff(phi3,t,2)  };
syms q1 q2 q3 q1dot q2dot q3dot q1ddot q2ddot q3ddot
newvars = { q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot };
L = subs(L, oldvars, newvars); % замена переменных в выражении L

% dL / dqdot
dLq1dot = diff(subs(L, oldvars, newvars), q1dot); % потому что нельзя написать diff(L, diff(phi1,t))
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


eq1 = dLq1dotdt - dLq1;
eq2 = dLq2dotdt - dLq2;
eq3 = dLq3dotdt - dLq3;

eq1 = simplify(eq1);
eq2 = simplify(eq2);
eq3 = simplify(eq3);


% Partial derivatives of Kinetic energy 
% syms dummy
% dLq1dot = subs(diff(subs(L, diff(q1,t),dummy),dummy),dummy,diff(q1,t));
% dLq2dot = subs(diff(subs(L, diff(q2,t),dummy),dummy),dummy,diff(q2,t));
% 
% % Partial derivatives of Lagrangian
% dLq1 = subs(diff(subs(L, q1, dummy),dummy),dummy, q1);
% dLq2 = subs(diff(subs(L, q2, dummy),dummy),dummy, q2);
% 
% % Lagrange Differential equation
% diffEq1 = diff(dLq1dot,t) - dLq1;
% diffEq2 = diff(dLq2dot,t) - dLq2;
% 
% % diffEq variables
% variables = { q1, q2,  diff(q1,t), diff(q2,t),  diff(q1,t,2), diff(q2,t,2)  };
% 
% % desired variables 
% syms q1 q2 q1dot q2dot q1ddot q2ddot
% variablesshort = { q1, q2, q1dot, q2dot, q1ddot, q2ddot  };
% 
% % Lagrange differential equations with desired variables
% DiffEq1 = subs(diffEq1, variables, variablesshort);
% DiffEq2 = subs(diffEq2, variables, variablesshort);
% 
% eq1 = simplify(DiffEq1);
% eq2 = simplify(DiffEq2);

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

% Gathering elements of matrix N from equations

N1 = subs(eq1, [q1ddot q2ddot], [0 0]);
N2 = subs(eq2, [q1ddot q2ddot], [0 0]);

N = [ N1; N2 ];

G1 = subs(eq1, [q1ddot q2ddot q1dot q2dot], [0 0 0 0]);
G2 = subs(eq2, [q1ddot q2ddot q1dot q2dot], [0 0 0 0]);

G = [ G1; G2 ];

V = expand(N - G);

% Solve system of L diff. equations relative to angular accelerations
% solution = solve(DiffEq1, DiffEq2,  q1ddot, q2ddot );
solution = solve(eq1, eq2,  q1ddot, q2ddot );

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;


Q1DDOT = simplify(Q1DDOT);
Q2DDOT = simplify(Q2DDOT);

time=[0 10];
% initial conditions ======================================================
x0=[ pi+0.1 0 0 0 ]; 


str=['xdot=@(t,x)[x(3);x(4);',char(Q1DDOT),';',char(Q2DDOT),'];'];
str = strrep(str, 'q1dot','x(3)');
str = strrep(str, 'q2dot','x(4)');
str = strrep(str, 'q1','x(1)');
str = strrep(str, 'q2','x(2)');
eval(str);

fig1 = figure(1);
clf('reset');
opts = odeset('Stats','on','OutputFcn',@odeplot);
[t,q]=ode23(xdot,time,x0,opts);


X1 = L1*sin(q(:,1));
Y1 = L1*cos(q(:,1));
X2 = L1*sin(q(:,1)) + L2*sin(q(:,2)+q(:,1));
Y2 = L1*cos(q(:,1)) + L2*cos(q(:,2)+q(:,1));


fig2 = figure(2);
clf('reset');
h = animatedline('Marker','o','LineWidth',1.5);
tr = animatedline();
axis equal;



for i=1:numel(q(:,1))-1
    
    addpoints(h, 0, 0);
    addpoints(h, X1(i), Y1(i));
    addpoints(h, X2(i), Y2(i));
    addpoints(tr, X2(i), Y2(i));

    
    drawnow;
%     pause(0.05);
    clearpoints(h);
end



