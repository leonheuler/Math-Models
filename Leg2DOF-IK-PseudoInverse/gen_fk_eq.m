

q = sym('q', [2, 1]);assume(q,'real');


L1 = 0.5;
L2 = 0.5;

A1 = [ 
       cos(q(1))    -sin(q(1));
       sin(q(1))    cos(q(1));
     ];
 
A2 = [
        cos(q(2))   sin(q(2)) ;
        -sin(q(2))  cos(q(2)) ;

      ];


rC = A1*[0; -L1];
rA = rC + A2*A1*[0; -L2];

rA = simplify(rA);
F = rA;

J = jacobian(F,q);
J = simplify(J);

matlabFunction(F,'File','f','Vars', {q});
matlabFunction(J,'File','jac','Vars', {q});





