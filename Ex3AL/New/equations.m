%% Derive equations of robot using MATLAB Symbolic Toolbox

%% Robot Parameters
 
% q = sym('q', [3 1]);
% L = sym('L', [3 1]);
syms q1 q2 q3
syms q1dot q2dot q3dot

% q1dot = diff(q1,t);
% q2dot = diff(q2,t);
% q3dot = diff(q3,t);


syms L1 L2 L3

A1 = Rz(q1);
T1 = A1;
r1 = [0; L1; 0; 1];
rc1 = [0; L1/2; 0; 1];

dT1dq1 = diff(T1, q1);
dTdq1q1dot = dT1dq1*q1dot;

v1 = mcrossv(dTdq1q1dot, r1);
vc1 = mcrossv(dTdq1q1dot, rc1);


A2 = T(0, L1, 0);
A3 = Rz(q2);
T2 = T1*A2*A3; T2 = combine(T2,'sincos');
r2 = [0; L2; 0; 1]; 
rc2 = [0; L2/2; 0; 1];

dT2dq1 = diff(T2, q1)*q1dot;
dT2dq2 = diff(T2, q2)*q2dot;

t1 = mcrossv(dT2dq1, r1);
t2 = mcrossv(dT2dq2, r2);   

v2 = v1 + mcrossv(dT2dq1, r2) + mcrossv(dT2dq2, r2);   


A4 = T(0, L2, 0);
A5 = Rz(q3);
T3 = T2*A4*A5; T3 = combine(T3,  'sincos');
r3 = T3*[0; L3; 0; 1];
rc3 = T3*[0; L3/2; 0; 1];

fig1 = figure(1);
clf('reset');
axis equal; grid on;
xyplane = animatedline('Marker','o');
c1p = animatedline('Marker','o','Color','red');
c2p = animatedline('Marker','o','Color','red');
c3p = animatedline('Marker','o','Color','red');

rc1 = double(subs(rc1, {q1 L1}, {0 0.5}));
rc2 = double(subs(rc2, {q1 q2 L1 L2}, {0 0 0.5 0.5}));
rc3 = double(subs(rc3, {q1 q2 q3 L1 L2 L3}, {0 0 0 0.5 0.5 0.75}));

r1 = double(subs(r1, {q1 L1}, {0 0.5}));
r2 = double(subs(r2, {q1 q2 L1 L2}, {0 0 0.5 0.5}));
r3 = double(subs(r3, {q1 q2 q3 L1 L2 L3}, {0 0 0 0.5 0.5 0.75}));

addpoints(xyplane, 0, 0);
addpoints(xyplane, r1(1), r1(2));
addpoints(xyplane, r2(1), r2(2));
addpoints(xyplane ,r3(1),  r3(2));

addpoints(c1p, rc1(1), rc1(2));
addpoints(c2p, rc2(1), rc2(2));
addpoints(c3p, rc3(1), rc3(2));

function ret = mcrossv(A,b)
   
    a1 = A(1,1:3).';
    a2 = A(2,1:3).';
    a3 = A(3,1:3).';
    c1 = cross(a1.',b(1:3));
    c2 = cross(a2.',b(1:3));
    c3 = cross(a3.',b(1:3));
    
    M = cat(1, c1, c2, c3);
    
    ret = blkdiag(M, 1);

end

        
