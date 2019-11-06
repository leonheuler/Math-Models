
% параметры
syms L1 L2 L3 m1 m2 m3 g
% L1 = 0.5;
% L2 = 0.5;
% L3 = 1;
% m1 = 3;
% m2 = 3;
% m3 = 5;
% g = 9.81;


% линейные положения точечных масс
syms t
syms q1(t) q2(t) q3(t)

x1 = L1*sin(q1);
y1 = L1*cos(q1);
x2 = L1*sin(q1) - L2*sin(q2-q1);
y2 = L1*cos(q1) + L2*cos(q2-q1);
x3 = L1*sin(q1) - L2*sin(q2-q1) - L3*sin(-q3+q2-q1);
y3 = L1*cos(q1) + L2*cos(q2-q1) + L3*cos(-q3+q2-q1);

% проекции линейных скоростей
x1dot = diff(x1,t);
y1dot = diff(y1,t);
x2dot = diff(x2,t);
y2dot = diff(y2,t);
x3dot = diff(x3,t);
y3dot = diff(y3,t);

% кинетическая энергия
K1 = m1/2*(x1dot^2+y1dot^2);
K2 = m2/2*(x2dot^2+y2dot^2);
K3 = m3/2*(x3dot^2+y3dot^2);
K = K1 + K2 + K3;

% потенциальная энергия
P1 = m1*g*L1*cos(q1);
P2 = m2*g*(L1*cos(q1) + L2*cos(q2-q1));
P3 = m3*g*(L1*cos(q1) + L2*cos(q2-q1) + L3*cos(-q3+q2-q1));

P = P1 + P2 + P3;

% лагранжиан
L = K - P;

% собственная частота
w = 25;
Kp = [w^2; w^2; w^2];
Kv = [2*w; 2*w; 2*w];

% частные производные кинетической энергии по обобщенным скоростям
syms dummy
dKq1dot = subs(diff(subs(K, diff(q1,t),dummy),dummy),dummy,diff(q1,t));
dKq2dot = subs(diff(subs(K, diff(q2,t),dummy),dummy),dummy,diff(q2,t));
dKq3dot = subs(diff(subs(K, diff(q3,t),dummy),dummy),dummy,diff(q3,t));

% частные производные лагранжиана по обобщенным координатам
dLq1 = subs(diff(subs(L, q1, dummy),dummy),dummy, q1);
dLq2 = subs(diff(subs(L, q2, dummy),dummy),dummy, q2);
dLq3 = subs(diff(subs(L, q3, dummy),dummy),dummy, q3);



% дифференциальные уравения движения
diffEq1 = diff(dKq1dot,t) - dLq1;
diffEq2 = diff(dKq2dot,t) - dLq2;
diffEq3 = diff(dKq3dot,t) - dLq3;



% переменные в дифф. ур-ях движения
variables = { q1, q2, q3, diff(q1,t), diff(q2,t), diff(q3,t), diff(q1,t,2), diff(q2,t,2), diff(q3,t,2) };

% желаемые переменные в дифф. ур-ях движения 
syms q1 q2 q3 q1dot q2dot q3dot q1ddot q2ddot q3ddot

% variablesshort = { x1, x2, x3, x4, x5, x6, q1ddot, q2ddot, q3ddot };
variablesshort = { q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot };

DiffEq1 = subs(diffEq1, variables, variablesshort);
DiffEq2 = subs(diffEq2, variables, variablesshort);
DiffEq3 = subs(diffEq3, variables, variablesshort);

eq1 = simplify(DiffEq1);
eq2 = simplify(DiffEq2);
eq3 = simplify(DiffEq3);

t11 = children(collect(eq1, q1ddot)); t12 = children(collect(eq1, q2ddot)); t13 = children(collect(eq1, q3ddot));
t21 = children(collect(eq2, q1ddot)); t22 = children(collect(eq2, q2ddot)); t23 = children(collect(eq2, q3ddot));
t31 = children(collect(eq3, q1ddot)); t32 = children(collect(eq3, q2ddot)); t33 = children(collect(eq3, q3ddot));

M11 = coeffs( t11(1), q1ddot); M12 = coeffs( t12(1), q2ddot); M13 = coeffs( t13(1), q3ddot);
M21 = coeffs( t21(1), q1ddot); M22 = coeffs( t22(1), q2ddot); M23 = coeffs( t23(1), q3ddot);
M31 = coeffs( t31(1), q1ddot); M32 = coeffs( t32(1), q2ddot); M33 = coeffs( t33(1), q3ddot);

M = [ M11 M12 M12;
      M21 M22 M23;
      M31 M32 M33 ];

% Решение диф.ур-й Лагранжа относительно ускорений ========================
solution = solve(DiffEq1, DiffEq2, DiffEq3, q1ddot, q2ddot, q3ddot);

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;
Q3DDOT = solution.q3ddot;
% 
% Q1DDOT = simplify(Q1DDOT);
% Q2DDOT = simplify(Q2DDOT);
% Q3DDOT = simplify(Q3DDOT);




