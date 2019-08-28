function EG()

tic

%%

function R = Rx(a)
    R = [1,    0,        0;
         0,   cos(a), -sin(a);
         0,   sin(a),  cos(a)];
end

function R = Ry(a)
    R = [cos(a),    0,        sin(a);
         0,         1,        0;
         -sin(a),   0,        cos(a)];
end

function ret = Rz(a)
    ret = [cos(a),    -sin(a),        0;
           sin(a),     cos(a),        0;
                0,          0,        1];
end

%%
M0 = 15;
M1 = 3;
M2 = 12;
M3 = 7;
M = M0 + 2*(M1+M2+M3);

%%
L0 = 0.443;

L1 = 0.270;
L3 = 0.42;
L5 = 0.475;
L7 = 0.227;
L9 = 0.16;

L2 = L1;
L4 = L3;
L6 = L5;
L8 = L7;
L10 = L9;

%%
q = sym('q', [12, 1]);assume(q,'real');

%%
p1 = [ q(10) q(11) q(12) ]';               % сдвиг локальной системы координат
R1 = Rz(q(9))*Ry(q(8))*Rx(q(7));     % ориентация локальной системы координат
% R1 = Ry(q(8));     % ориентация локальной системы координат
p2 = [L0/2 0 0]';             
rR1 = R1*p2 + p1;
rL1 = R1*(-p2) + p1;
% rR1 = R1*[L0/2 0 0]' + p1;   
% rL1 = R1*[-L0/2 0 0]' + p1;

%%
p3 = [0 L1 0]';
rR2 = R1*(Ry(q(2))*p3 + p2) + p1;
rL2 = R1*(Ry(q(1))*p3 - p2) + p1;
% rR2 = R1*(Ry(q2)*[0 L2 0]' + [L0/2 0 0]') + p1;
% rL2 = R1*(Ry(q1)*[0 L1 0]' + [-L0/2 0 0]') + p1;

%%
p4 = [0 0 -L3]';
rR3 = R1*(Ry(q(2))*(Rx(q(4))*p4 + p3) + p2) + p1;
rL3 = R1*(Ry(q(1))*(Rx(q(3))*p4 + p3) - p2) + p1;
% rR3 = R1*(Ry(q2)*(Rx(q4)*[0 0 -L4]' + [0 L2 0]') + [L0/2 0 0]') + p1;
% rL3 = R1*(Ry(q1)*(Rx(q3)*[0 0 -L3]' + [0 L1 0]') + [-L0/2 0 0]') + p1;

%%
p5 = [0 0 -L5]';
rR4 = R1*(Ry(q(2))*(Rx(q(4))*(Rx(q(6))*p5 + p4) + p3) + p2) + p1;
rL4 = R1*(Ry(q(1))*(Rx(q(3))*(Rx(q(5))*p5 + p4) + p3) - p2) + p1;
% rR4 = R1*(Ry(q2)*(Rx(q4)*(Rx(q6)*[0 0 -L6]' + [0 0 -L4]') + [0 L2 0]') + [L0/2 0 0]') + p1;
% rL4 = R1*(Ry(q1)*(Rx(q3)*(Rx(q5)*[0 0 -L5]' + [0 0 -L3]') + [0 L1 0]') + [-L0/2 0 0]') + p1;

%% 
c0  = (rR1 + rL1)/2;
cR1 = (rR1 + rR2)/2;
cR2 = (rR2 + rR3)/2;
cR3 = (rR3 + rR4)/2;
cL1 = (rL1 + rL2)/2;
cL2 = (rL2 + rL3)/2;
cL3 = (rL3 + rL4)/2;

C = (c0*M0 + (cR1+cL1)*M1 + (cR2+cL2)*M2 + (cR3+cL3)*M3 ) / M;

%%
F = [C; rR4; rL4];
F = simplify(F);
J = jacobian(F, q);
J = simplify(J);

matlabFunction(F,'File','f','Vars', {q});
matlabFunction(J,'File','jac','Vars', {q});

%%
toc

end