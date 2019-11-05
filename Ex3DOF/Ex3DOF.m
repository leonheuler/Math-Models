
% параметры
% syms L1 L2 L3 m1 m2 m3 g
L1 = 0.5;
L2 = 0.5;
L3 = 0.5;
m1 = 1;
m2 = 1;
m3 = 1;
g = 9.81;

% желаемые положения и скорости pi/3 pi/12 -pi/12
q1d = 0; q1dotd = 0; q1ddotd = 0;
q2d = 0; q2dotd = 0; q2ddotd = 0;
q3d = 0; q3dotd = 0; q3ddotd = 0;

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

% управляющие моменты
tau1 = -Kp(1)*(q1 - q1d) - Kv(1)*(diff(q1,t) - q1dotd );
tau2 = -Kp(2)*(q2 - q2d) - Kv(2)*(diff(q2,t) - q2dotd );
tau3 = -Kp(3)*(q3 - q3d) - Kv(3)*(diff(q3,t) - q3dotd );

% дифференциальные уравения движения
diffEq1 = diff(dKq1dot,t) - dLq1 + F(diff(q1,t)) - tau1;
diffEq2 = diff(dKq2dot,t) - dLq2 + F(diff(q2,t)) - tau2;
diffEq3 = diff(dKq3dot,t) - dLq3 + F(diff(q3,t)) - tau3;

% переменные в дифф. ур-ях движения
variables = { q1, q2, q3, diff(q1,t), diff(q2,t), diff(q3,t), diff(q1,t,2), diff(q2,t,2), diff(q3,t,2) };

% желаемые переменные в дифф. ур-ях движения 
syms x1 x2 x3 x4 x5 x6 q1ddot q2ddot q3ddot

variablesshort = { x1, x2, x3, x4, x5, x6, q1ddot, q2ddot, q3ddot };
variablesshort2 = { q1d, q2d, q3d, q1dotd, q2dotd, q3dotd, q1ddotd, q2ddotd, q3ddotd };

DiffEq1 = subs(diffEq1, variables, variablesshort);
DiffEq2 = subs(diffEq2, variables, variablesshort);
DiffEq3 = subs(diffEq3, variables, variablesshort);

tau1d = eval(subs(diffEq1, variables, variablesshort2));
tau2d = eval(subs(diffEq2, variables, variablesshort2));
tau3d = eval(subs(diffEq3, variables, variablesshort2));

% DiffEq1 = DiffEq1 - tau1d;
% DiffEq2 = DiffEq2 - tau2d;
% DiffEq3 = DiffEq3 - tau3d; 

disp([tau1d tau2d tau3d]);

% запись уравнений 
str1 = char(tau1);
str2 = char(tau2);
str3 = char(tau3);
str1 = strrep(str1, 'diff(q1(t),t)','x(4)'); str2 = strrep(str2, 'diff(q1(t),t)','x(4)'); str3 = strrep(str3, 'diff(q1(t),t)','x(4)');
str1 = strrep(str1, 'diff(q2(t),t)','x(5)'); str2 = strrep(str2, 'diff(q2(t),t)','x(5)'); str3 = strrep(str3, 'diff(q2(t),t)','x(5)');
str1 = strrep(str1, 'diff(q3(t),t)','x(6)'); str2 = strrep(str2, 'diff(q3(t),t)','x(6)'); str3 = strrep(str3, 'diff(q3(t),t)','x(6)');
str1 = strrep(str1, 'q1(t)','x(1)'); str2 = strrep(str2, 'q1(t)','x(1)'); str3 = strrep(str3, 'q1(t)','x(1)');
str1 = strrep(str1, 'q2(t)','x(2)'); str2 = strrep(str2, 'q2(t)','x(2)'); str3 = strrep(str3, 'q2(t)','x(2)');
str1 = strrep(str1, 'q3(t)','x(3)'); str2 = strrep(str2, 'q3(t)','x(3)'); str3 = strrep(str3, 'q3(t)','x(3)');
strtau = ['tau=@(x)[',str1,';',str2,';',str3,'];'];
eval(strtau);

% Решение диф.ур-й Лагранжа относительно ускорений ========================
solution = solve(DiffEq1, DiffEq2, DiffEq3, q1ddot, q2ddot, q3ddot);

Q1DDOT = solution.q1ddot;
Q2DDOT = solution.q2ddot;
Q3DDOT = solution.q3ddot;

Q1DDOT = simplify(Q1DDOT);
Q2DDOT = simplify(Q2DDOT);
Q3DDOT = simplify(Q3DDOT);

str1 = char(Q1DDOT);
str2 = char(Q2DDOT);
str3 = char(Q3DDOT);
str1 = strrep(str1, 'x1','x(1)'); str2 = strrep(str2, 'x1','x(1)'); str3 = strrep(str3, 'x1','x(1)');
str1 = strrep(str1, 'x2','x(2)'); str2 = strrep(str2, 'x2','x(2)'); str3 = strrep(str3, 'x2','x(2)');
str1 = strrep(str1, 'x3','x(3)'); str2 = strrep(str2, 'x3','x(3)'); str3 = strrep(str3, 'x3','x(3)');
str1 = strrep(str1, 'x4','x(4)'); str2 = strrep(str2, 'x4','x(4)'); str3 = strrep(str3, 'x4','x(4)');
str1 = strrep(str1, 'x5','x(5)'); str2 = strrep(str2, 'x5','x(5)'); str3 = strrep(str3, 'x5','x(5)');
str1 = strrep(str1, 'x6','x(6)'); str2 = strrep(str2, 'x6','x(6)'); str3 = strrep(str3, 'x6','x(6)');
strqddot = ['qddot=@(x)[',str1,';',str2,';',str3,'];'];
eval(strqddot);

time=[0 2];
% initial conditions ======================================================
x0=[pi/2+pi/12 pi/12 pi/12 0 0 0]; 
str=['xdot=@(t,x)[x(4);x(5);x(6);',char(Q1DDOT),';',char(Q2DDOT),';',char(Q3DDOT),'];'];
str = strrep(str, 'x1','x(1)');
str = strrep(str, 'x2','x(2)');
str = strrep(str, 'x3','x(3)');
str = strrep(str, 'x4','x(4)');
str = strrep(str, 'x5','x(5)');
str = strrep(str, 'x6','x(6)');

eval(str);

fig1 = figure(1);
clf('reset');
opts = odeset('Stats','on','OutputFcn',@odeplot);
[t,q]=ode45(xdot,time,x0,opts);

% вычислить положения звеньев как функции обобщенных координат
X1 = L1*sin(q(:,1));
Y1 = L1*cos(q(:,1));
X2 = L1*sin(q(:,1)) - L2*sin(q(:,2)-q(:,1));
Y2 = L1*cos(q(:,1)) + L2*cos(q(:,2)-q(:,1));
X3 = L1*sin(q(:,1)) - L2*sin(q(:,2)-q(:,1)) - L3*sin(-q(:,3)+q(:,2)-q(:,1));
Y3 = L1*cos(q(:,1)) + L2*cos(q(:,2)-q(:,1)) + L3*cos(-q(:,3)+q(:,2)-q(:,1));

for i=1:numel(q(:,1))-1
    qdd(i,1:3) = qddot(q(i,:))';
    moments(i,1:3) = double(tau(q(i,1:3)));
end

fig2 = figure(2);
clf('reset');
h = animatedline('Marker','o','LineWidth',1.5);
tr = animatedline();
axis equal;

fig3 = figure(3);
clf('reset');
e1plot = animatedline();
e2plot = animatedline();
e3plot = animatedline();

for i=1:numel(q(:,1))-1
    
    addpoints(h, 0, 0);
    addpoints(h, X1(i), Y1(i));
    addpoints(h, X2(i), Y2(i));
    addpoints(h, X3(i), Y3(i));
    addpoints(tr, X3(i), Y3(i));
    addpoints(e1plot,i, q(i,1)-q1d );
    addpoints(e2plot,i, q(i,2)-q2d );
    addpoints(e3plot,i, q(i,3)-q3d );
    
    drawnow;
%     pause(0.05);
    clearpoints(h);
end

addpoints(h, 0, 0);
addpoints(h, X1(numel(q(:,1))), Y1(numel(q(:,1))));
addpoints(h, X2(numel(q(:,1))), Y2(numel(q(:,1))));
addpoints(h, X3(numel(q(:,1))), Y3(numel(q(:,1))));

function ret = F(q_dot)
    
    w = q_dot;
    brkwy_trq = 2;        
    brkwy_vel = 0.01;   
    Col_trq = 1.4;
    visc_coef = 0.01;
    
    static_scale = sqrt(2*exp(1))*(brkwy_trq-Col_trq);
    static_thr = sqrt(2)*brkwy_vel;                     % Velocity threshold for static torque
    Col_thr = brkwy_vel/10;    
    
    ret = visc_coef * w ...
         + static_scale * (w/static_thr*exp(-(w/static_thr)^2)) ...
         + Col_trq * tanh(w/Col_thr); 
end
