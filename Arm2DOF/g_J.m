syms L1 L2
q = sym('q', [2, 1]);assume(q,'real');

rA = [ L1*cos(q(1)) + L2*cos(q(1)+q(2));
       L1*sin(q(1)) + L2*sin(q(1)+q(2)) ];



J = jacobian(rA, q);

Jinv = inv(J);
Jinv = simplify(Jinv);