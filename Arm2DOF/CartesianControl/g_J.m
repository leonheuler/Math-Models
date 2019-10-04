% syms L1 L2 
L1 = 0.5; L2 = 0.5;
m1 = 3; m2 = 3;
% syms m1 m2
% syms g 
g = 9.81;
q = sym('q', [2, 1]);assume(q,'real');
q_dot = sym('q_dot', [2, 1]); assume(q_dot, 'real');
y_dot = sym('y_dot', [2, 1]); assume(y_dot,'real');


rA = [ L1*cos(q(1)) + L2*cos(q(1)+q(2));
       L1*sin(q(1)) + L2*sin(q(1)+q(2)) ];



J = jacobian(rA, q);
matlabFunction(J, 'File', 'J', 'Vars', {q});
% trJ = transpose(J);
% matlabFunction(trJ,'File','trJ','Vars', {q});
% 
% invJ = inv(J);
% invJ = simplify(invJ);
% tinvJ = transpose(invJ);
% matlabFunction(invJ,'File','invJ','Vars', {q});
% matlabFunction(tinvJ,'File','tinvJ','Vars', {q});
% 
% 
% dotJ = [ -L1*cos(q(1))*q_dot(1) - L2*cos(q(1) + q(2))*(q_dot(1) + q_dot(2)), -L2*cos(q(1)+q(2))*(q_dot(1) + q_dot(2));
%          -L1*sin(q(1))*q_dot(1) - L2*sin(q(1) + q(2))*(q_dot(1) + q_dot(2)), -L2*sin(q(1)+q(2))*(q_dot(1) + q_dot(2))];
% matlabFunction(dotJ,'File','dotJ','Vars', {q,q_dot});     
%     
% M = [ (m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q(2)),      m2*L2^2+m2*L1*L2*cos(q(2));
%         m2*L2^2+m2*L1*L2*cos(q(2)),                         m2*L2^2];
% 
% 
% V =  [ -m2*L1*L2*(2*q_dot(1)*q_dot(2)+(q_dot(2)^2))*sin(q(2));
%         m2*L1*L2*(q_dot(1)^2)*sin(q(2))];
% 
% 
% 
% G = [(m1+m2)*g*L1*cos(q(1)) + m2*g*L2*cos(q(1)+q(2));
%         m2*g*L2*cos(q(1)+q(2))];
% 
% N = V + G;
%     
%     
% M = simplify(tinvJ*M*invJ);
% invM = simplify(inv(M));
% matlabFunction(invM,'File','invM','Vars', {q});
% 
% N = simplify(tinvJ*(N - M*invJ*dotJ*q_dot));
% 
% matlabFunction(M,'File','M','Vars', {q});
% matlabFunction(N,'File','N','Vars', {q, q_dot});


    
    
    
    
    
    
    
    
    
    
    
    